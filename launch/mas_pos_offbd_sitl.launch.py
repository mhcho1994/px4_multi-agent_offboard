from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

# def generate_launch_description():
    # gz_world_name = LaunchConfiguration('gz_world_name')
    # gz_model_names = LaunchConfiguration('gz_model_names')

    # gz_world_name_arg = DeclareLaunchArgument(
    #     'gz_world_name',
    #     default_value='AbuDhabiSwarm'
    # )
    # gz_model_names_arg = DeclareLaunchArgument(
    #     'gz_model_names',
    #     default_value="[x500_1, x500_2, cf1, cf2, cf3]"
    # )
    
    # interagent_meas_publisher = Node(
    #     package='gz_interagent_meas',
    #     executable='interagent_range_pub',
    #     parameters=[
    #         {'gz_world_name': gz_world_name},
    #         {'gz_model_names': gz_model_names},
    #     ]
    # )

    # return LaunchDescription([
    #     gz_world_name_arg,
    #     gz_model_names_arg,
    #     interagent_meas_publisher,
    # ])