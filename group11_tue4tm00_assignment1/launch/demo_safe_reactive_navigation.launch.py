
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
                    os.path.join(get_package_share_directory('core_tue4tm00_robot_simulate'), 'launch', 'robocyl_goal_gazebo_turtlebot3_stage2.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
        },
    )
    ld.add_action(robocyl_gazebo_launch)

    safe_reactive_navigation_launch = TimerAction(
        period=5.0, # Delay in seconds
        actions = [
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('group11_tue4tm00_assignment1'), 'launch', 'safe_reactive_navigation.launch.py')
                        )
                    )
                ],
                scoped=True,
                forwarding=False,
                launch_configurations={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'namespace': LaunchConfiguration('namespace'),
                    'pose': ['/mocap/', LaunchConfiguration('namespace'), '/pose'],
                    'goal': '/mocap/goal/pose',
                    'cmd_vel': 'cmd_vel_ctrl',
                },
            )
        ]
    )
    ld.add_action(safe_reactive_navigation_launch)

    return ld
