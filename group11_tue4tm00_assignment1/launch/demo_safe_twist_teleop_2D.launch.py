
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use Gazebo simulation clock if true')
    ld.add_action(use_sim_time)
    namespace = DeclareLaunchArgument('namespace', default_value='robocyl', description='RoboCyl name')
    ld.add_action(namespace)
    
    robocyl_gazebo_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_tue4tm00_robot_simulate'), 'launch', 'robocyl_gazebo_turtlebot3_stage2.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'cmd_vel_key': 'cmd_vel_key_raw',
        },
    )
    ld.add_action(robocyl_gazebo_launch)

    safe_teleop_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('group11_tue4tm00_assignment1'), 'launch', 'safe_twist_teleop_2D.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'cmd_vel_in': 'cmd_vel_key_raw',
            'cmd_vel_out': 'cmd_vel_key',
            'scan': 'scan',
            'pose': ['/mocap/', LaunchConfiguration('namespace'), '/pose'],
        },
    )
    ld.add_action(safe_teleop_launch)

    return ld
