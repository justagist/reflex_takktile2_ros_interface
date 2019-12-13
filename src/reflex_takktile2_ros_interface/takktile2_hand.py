import rospy
from std_msgs.msg import Float32

# /***************************************************************************
# Copyright (c) 2019, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/
import numpy as np
import quaternion

import logging

from reflex_msgs2.msg import PoseCommand, ForceCommand, VelocityCommand, Hand
from std_srvs.srv import Empty
import reflex_msgs2.srv

from reflex_takktile2_ros_interface.config import takktile2_config

import threading


class Takktile2Hand():

    def __init__(self, robot_name = "takktile2_hand", on_state_callback=None):
        """
        Class constructor
        Args: 
        robot_name: a string (ideally unique and human readable) representing this robot name
        on_state_callback: an optional callback
        Returns:
        none, store the trajectories
        """

        self._logger = logging.getLogger(__name__)

        self._ready = False

        self._logger.info("Waiting for Reflex Driver Node...")
        rospy.wait_for_service('/reflex_takktile2/enable_tactile_stops')
        self._logger.info("Reflex Driver Running...")

        # Configuring hand (setting up publishers, variables, etc)
        self._configure(robot_name, on_state_callback)

        self._ready = True  # Hand is ready to be used
        self._lock = threading.Lock()
        self._preshape = 3.14/6

    def _update_state(self, new_state = None):

        with self._lock:
            self._state = {}
            now                 = rospy.Time.now()
            self._state['timestamp'] = {'secs': now.secs, 'nsecs': now.nsecs}

            self._state['position']        = np.asarray([m.joint_angle for m in new_state.motor])
            self._state['velocity']        = np.asarray([m.velocity for m in new_state.motor])
            self._state['effort']          = np.asarray([m.load for m in new_state.motor])

            self._state['contact_status']  = np.asarray([m.contact for m in new_state.finger])
            self._state['pressure']  = np.asarray([m.pressure for m in new_state.finger])

            self._state['proximal'] = np.asarray([m.proximal for m in new_state.finger])
            self._state['distal_approx'] = np.asarray([m.distal_approx for m in new_state.finger])

            self._state['finger_imu_quat'] = np.asarray([m.imu.quat for m in new_state.finger])
            self._state['finger_imu_eul'] = np.asarray([m.imu.euler_angles for m in new_state.finger])

            self._state['palm_imu_quat'] = np.asarray(new_state.palmImu.quat)
            self._state['palm_imu_eul'] = np.asarray(new_state.palmImu.euler_angles)

            self._on_state_callback(self._state)


    def _configure(self, limb, on_state_callback):


        self.name = limb


        self._config = takktile2_config

        self._all_joint_names = []
        self._all_joint_names = self._config["all_joint_names"]

        self._state = None

        if on_state_callback:
            self._on_state_callback = on_state_callback
        else:
            self._on_state_callback = lambda m: None

        self._pos_cmd_pub = rospy.Publisher('/reflex_takktile2/command_position', PoseCommand, queue_size=10)
        self._vel_cmd_pub = rospy.Publisher('/reflex_takktile2/command_velocity', VelocityCommand, queue_size=10)
        self._force_cmd_pub = rospy.Publisher('/reflex_takktile2/command_motor_force', ForceCommand, queue_size=10)
        self._hand_state_subscriber = rospy.Subscriber('/reflex_takktile2/hand_state', reflex_msgs2.msg.Hand, self._receive_hand_state_cb)

        self._finger_calibration_request = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
        self._tactile_calibration_request = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
        self._enable_tactile_stop_request = rospy.ServiceProxy('/reflex_takktile2/enable_tactile_stops', Empty)
        self._disable_tactile_stop_request = rospy.ServiceProxy('/reflex_takktile2/disable_tactile_stops', Empty)
        self._set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile2/set_tactile_threshold', reflex_msgs2.srv.SetTactileThreshold)


        self._tactile_stops_enabled = False

        self._nq = len(self._all_joint_names)
        self._nu = len(self._all_joint_names)

        self._jnt_limits = self._config['joint_limits']
        self._q_mean = np.array([0.5 * (limit['lower'] + limit['upper']) for limit in self._jnt_limits])


    def _receive_hand_state_cb(self, msg):
        if self._ready:
            self._update_state(new_state = msg)

    def exec_position_cmd(self, cmd):
        preshape = self._preshape if len(cmd) < 4 else cmd[3]
        self._pos_cmd_pub.publish(f1 = cmd[0], f2 = cmd[1], f3 = cmd[2], preshape = preshape)

    def exec_velocity_cmd(self, cmd):
        preshape = self._preshape if len(cmd) < 4 else cmd[3]
        self._vel_cmd_pub.publish(f1 = cmd[0], f2 = cmd[1], f3 = cmd[2], preshape = cmd[3])

    def exec_force_cmd(self, cmd, timeout = 5.0):

        preshape = self._preshape if len(cmd) < 4 else cmd[3]
        self._force_cmd_pub.publish(f1 = cmd[0], f2 = cmd[1], f3 = cmd[2], preshape = preshape)

        end_time = rospy.Time.now().to_sec() + timeout

        while rospy.Time.now().to_sec() < end_time:
            if np.allclose(self.joint_velocities(), np.zeros(4), atol = 0.001):
                self._logger.debug("Force control successful")
                return

        self._logger.warn("Force control timed out")
        self.exec_position_cmd(self.angles())

    def move_to_joint_position(self, cmd):
        self.exec_position_cmd(cmd)

    def angles(self):
        return self._state['position']

    def joint_velocities(self):
        return self._state['velocity']

    def joint_efforts(self):
        return self._state['effort']

    def q_mean(self):
        return self._q_mean

    def n_cmd(self):
        return self._nu

    def n_joints(self):
        return self._nq


    def tuck(self):
        self.move_to_joint_position(self._q_mean)

    def untuck(self):
        self.move_to_joint_position([j['lower'] for j in self._jnt_limits])

    def joint_names(self):
        return self._all_joint_names

    def joint_limits(self):
        return self._jnt_limits

    def state(self):
        with self._lock:
            return self._state

    def calibrate_fingers(self):
        self._logger.info("Calibrating fingers...")
        self._finger_calibration_request()

    def calibrate_tactile(self):
        self._logger.info("Calibrating tactile sensors...")
        self._tactile_calibration_request()

    def enable_tactile_stops(self, enable = True):

        self._tactile_stops_enabled = enable
        if enable:
            self._enable_tactile_stop_request()
        else:
            self._disable_tactile_stop_request()

    def set_tactile_threshold(self, threshold_list):
        '''
            @param threshold_list: list of list of shape 3 x 14 (14 sensors per finger).
        '''
        self._set_tactile_threshold(threshold_list)

    def calibrate(self):

        self.calibrate_tactile()
        rospy.sleep(3.0)
        self.calibrate_fingers()

    def set_preshape(self, value):
        self._preshape = value
        angles = self.angles()
        angles[3] = self._preshape
        self.exec_position_cmd(angles)

