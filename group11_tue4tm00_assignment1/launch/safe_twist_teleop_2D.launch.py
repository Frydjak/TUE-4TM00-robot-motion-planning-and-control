from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    ld = LaunchDescription()

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    ld.add_action(use_sim_time)
    namespace = DeclareLaunchArgument('namespace', default_value='myrobot') 
    ld.add_action(namespace)
    default_config = os.path.join(get_package_share_directory('group11_tue4tm00_assignment1'), 'config', 'safe_twist_teleop_2D.yaml') 
    config = DeclareLaunchArgument('config', default_value=default_config)
    ld.add_action(config)
    cmd_vel_in = DeclareLaunchArgument('cmd_vel_in', default_value='cmd_vel_in')
    ld.add_action(cmd_vel_in)
    cmd_vel_out = DeclareLaunchArgument('cmd_vel_out', default_value='cmd_vel_out')
    ld.add_action(cmd_vel_out)
    scan = DeclareLaunchArgument('scan', default_value='scan')
    ld.add_action(scan)
    pose = DeclareLaunchArgument('pose', default_value='pose')
    ld.add_action(pose)

    safe_twist_teleop_node = Node(
        package='group11_tue4tm00_assignment1',
        executable='safe_twist_teleop_2D.py',
        name='safe_twist_teleop',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('cmd_vel_in', LaunchConfiguration('cmd_vel_in')),
            ('cmd_vel_out', LaunchConfiguration('cmd_vel_out')),
            ('scan', LaunchConfiguration('scan')),
            ('pose', LaunchConfiguration('pose')),
        ],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(safe_twist_teleop_node)

    return ld