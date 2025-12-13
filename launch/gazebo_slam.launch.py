from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('slamcore').find('slamcore')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'slam_world.sdf'])
    robot_model = PathJoinSubstitution([pkg_share, 'models', 'slam_robot.urdf'])
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='World file path'
    )
    
    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start GUI'
    )
    
    gazebo_server = Node(
        package='gazebo_ros',
        executable='gazebo',
        arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                   LaunchConfiguration('world')],
        output='screen'
    )
    
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[robot_model],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    lidar_odometry = Node(
        package='slamcore',
        executable='lidar_odometry_node',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])],
        output='screen'
    )
    
    visual_odometry = Node(
        package='slamcore',
        executable='visual_odometry_node',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])],
        output='screen'
    )
    
    imu_integration = Node(
        package='slamcore',
        executable='imu_integration_node',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])],
        output='screen'
    )
    
    loop_closure = Node(
        package='slamcore',
        executable='loop_closure_node',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])],
        output='screen'
    )
    
    map_builder = Node(
        package='slamcore',
        executable='map_builder_node',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])],
        output='screen'
    )
    
    backend_optimizer = Node(
        package='slamcore',
        executable='backend_optimizer_node',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])],
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_arg,
        declare_gui_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        lidar_odometry,
        visual_odometry,
        imu_integration,
        loop_closure,
        map_builder,
        backend_optimizer,
    ])
