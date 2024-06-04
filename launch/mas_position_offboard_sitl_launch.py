import os
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import yaml
from pathlib import Path
import numpy as np


def generate_launch_description():

	fileDir 	= 	os.path.dirname(os.path.realpath('__file__'))
	parent_dir 	=	os.path.abspath(os.path.join(fileDir, os.pardir))

	yaml_dict 	= 	yaml.safe_load(Path(os.path.join("/home/user/work/ros2_ws/src/px4-multiagent-offboard","config/mixedexp_3d_10a.yaml")).read_text())

	ros_ns_str 	=	yaml_dict['ros_ns_str']

	formation   =   np.zeros((len(yaml_dict['ros_ns_str'])*3),dtype=np.float64)

	for index, x in np.ndenumerate(formation):
		if isinstance(yaml_dict['formation'][index[0]], str):
			formation[index[0]]	=   eval(yaml_dict['formation'][index[0]])

		elif isinstance(yaml_dict['formation'][index[0]], float) or isinstance(yaml_dict['formation'][index[0]], int):
			formation[index[0]]	=   np.float64(yaml_dict['formation'][index[0]])

	formation 	=	np.ndarray.tolist(formation)

	adjacency   =   np.zeros((len(yaml_dict['ros_ns_str'])*len(yaml_dict['ros_ns_str'])),dtype=np.float64)

	for index, x in np.ndenumerate(adjacency):
		if isinstance(yaml_dict['adjacency'][index[0]], float) or isinstance(yaml_dict['adjacency'][index[0]], int):
			adjacency[index[0]]	=   np.float64(yaml_dict['adjacency'][index[0]])

	adjacency 	=	np.ndarray.tolist(adjacency)

	mas_offboard_position_ctrl_node = Node(
		package='px4-multiagent-offboard',
	    executable='mas_position_offboard_mxexp.py',
	    name='mas_position_offboard_mxexp',
		parameters = [
			{'ros_ns' : ros_ns_str},
			{'formation' : formation},
			{'adjacency' : adjacency}
		])
	
	visualizer = Node(
		package='px4-multiagent-offboard',
	    executable='visualizer_real.py',
	    name='visualizer_real',
		parameters = [
		])

	return LaunchDescription([
		mas_offboard_position_ctrl_node,
		visualizer
		])