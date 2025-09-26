import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(bringup_dir, 'urdf', 'Assem2.SLDASM.urdf'))
    rviz_enabled = LaunchConfiguration('rviz_enabled', default='True')

    # Read URDF
    with open(os.path.join(bringup_dir, 'urdf', 'Assem2.SLDASM.urdf'), 'r') as infp:
        robot_desc = infp.read()

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(bringup_dir, 'world'),
            str(Path(bringup_dir).parent.resolve())
        ])
    )

    # Start Gazebo simulation
    world = os.path.join(bringup_dir , "world", "depot.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 4 {world}"}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robot",
            "-topic", "/robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.2",  # slight lift
        ],
        output="screen",
    )

    # Delay robot_state_publisher to ensure joint_states exist
    start_robot_state_publisher_cmd = TimerAction(
        period=2.0,  # wait 2 seconds
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='both',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_description': robot_desc}
                ]
            )
        ]
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Static map->odom transform
    tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    # RViz node
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'robot.rviz')  # make sure you have an rviz config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(rviz_enabled)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time'),
        DeclareLaunchArgument('urdf_file', default_value=os.path.join(bringup_dir, 'urdf', 'Assem2.SLDASM.urdf'), description='URDF file path'),
        DeclareLaunchArgument('rviz_enabled', default_value='True', description='Launch RViz if true'),
        gz_resource_path,
        gz_sim,
        spawn_entity,
        start_robot_state_publisher_cmd,
        bridge,
        tf_map,
        rviz_node
    ])
