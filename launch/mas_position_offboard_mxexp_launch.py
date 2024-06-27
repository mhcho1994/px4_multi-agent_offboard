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

	config = os.path.join(
		get_package_share_directory('px4-multiagent-offboard'),
		'config',
		'mixedexp_3d_10a.yaml'
	)

	mas_offboard_position_ctrl_node = Node(
		package='px4-multiagent-offboard',
	    executable='mas_position_offboard_mxexp.py',
	    name='mas_position_offboard_mxexp',
		parameters = [
			config
		])
	
	visualizer = Node(
		package='px4-multiagent-offboard',
	    executable='visualizer_real.py',
	    name='visualizer_real'
		)

	return LaunchDescription([
		mas_offboard_position_ctrl_node,
		visualizer
		])