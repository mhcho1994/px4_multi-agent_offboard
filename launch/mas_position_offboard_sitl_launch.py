import os
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource



def generate_launch_description():

	# ros_ns_str = ['px4_1', 'px4_2', 'px4_3', 'px4_4', 'px4_5', 'px4_6', 'px4_7']

	# ros_ns_str_cpp = '['
	# for str in ros_ns_str:
	# 	ros_ns_str_cpp += str
	# 	ros_ns_str_cpp += ','
	# ros_ns_str_cpp += ']'

	mas_offboard_position_ctrl_node = Node(
		package='px4-multiagent-offboard',
	    executable='mas_position_offboard_mxexp.py',
	    name='mas_position_offboard_mxexp',
		parameters = [
		])
	
	visualizer = Node(
		package='px4-multiagent-offboard',
	    executable='visualizer_real.py',
	    name='visualizer_real',
		parameters = [
		])

	return LaunchDescription([
		mas_offboard_position_ctrl_node,
		visualizer])
