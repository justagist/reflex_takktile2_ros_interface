from reflex_takktile2_ros_interface import Takktile2Hand
import rospy

if __name__ == '__main__':
	rospy.init_node("test")
	hand = Takktile2Hand()

	