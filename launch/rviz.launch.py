from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('slamcore').find('slamcore')
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz'])
    
    declare_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='RViz configuration file'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_config_arg,
        rviz_node,
    ])
