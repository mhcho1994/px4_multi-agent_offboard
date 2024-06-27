#!/usr/bin/env python3

import yaml
import os
from pathlib import Path
import numpy as np

file_dir 	= 	os.path.dirname(os.path.realpath('__file__'))
parent_dir 	=	os.path.abspath(os.path.join(file_dir, os.pardir))

yaml_dict = yaml.safe_load(Path(os.path.join(parent_dir,"config/mixedexp_3d_10a.yaml")).read_text())

# print(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation'])
# print(type(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation']))

formation   =   np.zeros((len(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation']),3),dtype=np.float64)

for index, x in np.ndenumerate(formation):
    if isinstance(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation'][index[0]][index[1]], str):
        print(x)
        print(index)
        formation[index[0],index[1]]    =   eval(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation'][index[0]][index[1]])

    elif isinstance(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation'][index[0]][index[1]], float) or isinstance(yaml_dict['formation'][index[0]][index[1]], int):
        formation[index[0],index[1]]    =   np.float64(yaml_dict['mas_position_offboard_mxexp']['ros__parameters']['formation'][index[0]][index[1]])

formation 	=	np.ndarray.tolist(formation)

print(formation)

# adjacency   =   np.zeros((len(yaml_dict['ros_ns_str']),len(yaml_dict['ros_ns_str'])),dtype=np.float64)

# for index, x in np.ndenumerate(adjacency):
#     if isinstance(yaml_dict['adjacency'][index[0]][index[1]], float) or isinstance(yaml_dict['adjacency'][index[0]][index[1]], int):
#         adjacency[index[0],index[1]]    =   np.float64(yaml_dict['adjacency'][index[0]][index[1]])

# adjacency 	=	np.ndarray.tolist(adjacency)

# print(adjacency)
# print(type(adjacency))
# print(yaml_dict['formation'][index[0]][index[1]])
    # print(type(yaml_dict['formation'][index[0]][index[1]]))
    # print(type(x))

# print(formation)



	# yaml_dict 	= 	yaml.safe_load(Path(os.path.join("/home/user/work/ros2_ws/src/px4-multiagent-offboard","config/mixedexp_3d_10a.yaml")).read_text())

	# ros_ns_str 	=	yaml_dict['ros_ns_str']

	# formation   =   np.zeros((len(yaml_dict['ros_ns_str'])*3),dtype=np.float64)

	# for index, x in np.ndenumerate(formation):
	# 	if isinstance(yaml_dict['formation'][index[0]], str):
	# 		formation[index[0]]	=   eval(yaml_dict['formation'][index[0]])

	# 	elif isinstance(yaml_dict['formation'][index[0]], float) or isinstance(yaml_dict['formation'][index[0]], int):
	# 		formation[index[0]]	=   np.float64(yaml_dict['formation'][index[0]])

	# formation 	=	np.ndarray.tolist(formation)

	# adjacency   =   np.zeros((len(yaml_dict['ros_ns_str'])*len(yaml_dict['ros_ns_str'])),dtype=np.float64)

	# for index, x in np.ndenumerate(adjacency):
	# 	if isinstance(yaml_dict['adjacency'][index[0]], float) or isinstance(yaml_dict['adjacency'][index[0]], int):
	# 		adjacency[index[0]]	=   np.float64(yaml_dict['adjacency'][index[0]])

	# 