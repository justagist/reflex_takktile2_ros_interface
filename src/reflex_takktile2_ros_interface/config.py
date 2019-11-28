namespace = "reflex_takktile2"

takktile2_config = {}

all_joint_names = ["%s_f1"%namespace,
				   "%s_f2"%namespace,
				   "%s_f3"%namespace,
				   "%s_preshape"%namespace]

joint_limits = [{'lower': 0.0, 'upper': 3.14},
	            {'lower': 0.0, 'upper': 3.14},
	            {'lower': 0.0, 'upper': 3.14},
	            {'lower': 0.0, 'upper': 3.14/2}]

takktile2_config['all_joint_names'] = all_joint_names
takktile2_config['joint_limits'] = joint_limits