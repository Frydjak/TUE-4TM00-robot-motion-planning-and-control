
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
    namespace = DeclareLaunchArgument('namespace', default_value='turtlebot', description='robot name')
    ld.add_action(namespace)
    

    rviz_config = os.path.join(get_package_share_directory('group11_tue4tm00_project'), 'config', 'robot_goal_scan_map_costmap_path.rviz')
    turtlebot_map_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_tue4tm00_turtlebot3_simulate'), 'launch', 'turtlebot3_map_gazebo_core_large_office.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'pose_rate': '0.3',
            'rviz_config': rviz_config,
        },
    )
    ld.add_action(turtlebot_map_launch)

    goal_file = os.path.join(get_package_share_directory('group11_tue4tm00_project'), 'config', 'goal_poses.yaml')
    goal_publisher_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_goal_tools'), 'launch', 'goal_pose_publisher.launch.py')
                ),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'goal_tolerance': '0.5',
            'goal_file': goal_file,
            'goal_topic': 'goal',
            'pose_topic': ['/mocap/', LaunchConfiguration('namespace'), '/pose'],
        },
    )
    ld.add_action(goal_publisher_launch)

    odometry_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('group11_tue4tm00_project'), 
                                 'launch', 'odometry.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'pose_in': ['/mocap/', LaunchConfiguration('namespace'), '/pose'], 
            'cmd_vel': 'cmd_vel',
            'pose_out': 'odom_pose',
        },
    )
    ld.add_action(odometry_launch)


    costmap_launch = TimerAction(
        period=0.0, # Delay in seconds
        actions = [
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('group11_tue4tm00_project'), 'launch', 'costmap.launch.py')
                        )
                    )
                ],
                scoped=True,
                forwarding=False,
                launch_configurations={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'namespace': LaunchConfiguration('namespace'),
                    'pose': 'odom_pose',
                    'goal': 'goal',
                    'scan': 'scan',
                    'map': 'map',
                    'costmap': 'costmap',
                },
            )
        ]
    )
    ld.add_action(costmap_launch)

    path_planner_launch = TimerAction(
        period=3.0, # Delay in seconds
        actions = [
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('group11_tue4tm00_project'), 'launch', 'path_planner.launch.py')
                        )
                    )
                ],
                scoped=True,
                forwarding=False,
                launch_configurations={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'namespace': LaunchConfiguration('namespace'),
                    'pose': 'odom_pose',
                    'goal': 'goal',
                    'scan': 'scan',
                    'map': 'map',
                    'costmap': 'costmap',
                    'path': 'path',
                },
            )
        ]
    )
    ld.add_action(path_planner_launch)

    path_follower_launch = TimerAction(
        period=3.0, # Delay in seconds
        actions = [
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('group11_tue4tm00_project'), 'launch', 'path_follower.launch.py')
                        )
                    )
                ],
                scoped=True,
                forwarding=False,
                launch_configurations={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'namespace': LaunchConfiguration('namespace'),
                    'pose': 'odom_pose',
                    'goal': 'goal',
                    'scan': 'scan',
                    'map': 'map',
                    'costmap': 'costmap',
                    'path': 'path',
                    'cmd_vel': 'cmd_vel_ctrl',
                },
            )
        ]
    )
    ld.add_action(path_follower_launch)

    return ld
