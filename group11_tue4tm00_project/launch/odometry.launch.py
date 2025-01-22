from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    namespace = DeclareLaunchArgument('namespace', default_value='turtlebot')
    pose_in = DeclareLaunchArgument('pose_in', default_value='pose_in')
    cmd_vel = DeclareLaunchArgument('cmd_vel', default_value='cmd_vel')
    pose_out = DeclareLaunchArgument('pose_out', default_value='pose_out')
    
    default_config = os.path.join(get_package_share_directory('group11_tue4tm00_project'),'config', 'odometry.yaml')
    config = DeclareLaunchArgument('config', default_value=default_config, 
                                   description='config file for node parameters')

    odometry_node = Node(
        package='group11_tue4tm00_project',
        executable='odometry.py',
        name='odometry',
        namespace = LaunchConfiguration('namespace'),
        parameters = [
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('pose_in', LaunchConfiguration('pose_in')),
            ('cmd_vel', LaunchConfiguration('cmd_vel')),
            ('pose_out', LaunchConfiguration('pose_out')), 
        ],
        output='screen',
        emulate_tty=True,
    )
                          
    return LaunchDescription([
        use_sim_time,
        namespace,
        pose_in,
        cmd_vel,
        pose_out,
        config,
        odometry_node
        ])