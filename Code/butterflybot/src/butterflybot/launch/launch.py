import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    sdf_file_name = 'butterflybot.sdf'
    rviz_config = 'butterflybot.rviz'

    urdf_robot_file = 'butterflybot.urdf'  # or butterflybot.xacro
    path_to_urdf_robot = os.path.join(
    get_package_share_directory('butterflybot'),
    'urdf',
    urdf_robot_file)

    path_to_sdf = os.path.join(
        get_package_share_directory('butterflybot'),
        'urdf',
        sdf_file_name)

    path_to_world = os.path.join(
        get_package_share_directory('butterflybot'),
        'urdf',
        'env_ws.sdf')

    path_to_rviz = os.path.join(
        get_package_share_directory('butterflybot'),
        'rviz',
        rviz_config)

    path_to_config = os.path.join(
        get_package_share_directory('butterflybot'),
        'config'
    )

    world_launch = f"-r -v 4 {path_to_world}"

    resource_path = get_package_share_directory('butterflybot')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(open(path_to_urdf_robot, 'r').read(), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": world_launch}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robot1",
            "-file", path_to_sdf,
            "-x", "-1", "-y", "0", "-z", "0.5"
        ],
        output="screen",
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Joint States
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/robot1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Propellor Joint Data
            '/joint_front_left_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint_front_right_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint_rear_left_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint_rear_right_topic@std_msgs/msg/Float64@gz.msgs.Double',
            # Propellor Thrust Data
            '/model/robot1/joint/joint_front_left/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/joint_front_right/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/joint_rear_left/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/joint_rear_right/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
             # Bridge velocity commands (if controlling with velocity) between ROS 2 and Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            # Sensor Data
            '/boxes_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lidar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Visualization Marker Array Bridge
            '/visualization_marker_array@visualization_msgs/msg/MarkerArray[gz.msgs.Marker',
            # Navigation Topic
            #'/map@nav_msgs/msg/OccupancyGrid[gz.msgs.OccupancyGrid',
            #'/path@nav_msgs/msg/Path[gz.msgs.Path',
            #'/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF Data
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # Add TF broadcaster to send transforms for base_link and camera_link
    transform_broadcaster_node = Node(
        package='butterflybot',  # Your package name
        executable='transform_broadcaster',  # The Python script you created
        name='transform_broadcaster_node',
        output='screen'
    )

    # Static Transform Publisher for map to world
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_broadcaster',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'map',
            '--child-frame-id', 'world'
        ]
    )

    # Static transform from base_link to lidar_link
    static_lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_broadcaster',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'robot1/lidar_link/gpu_lidar'
        ]
    )

    # Static transform from base_link to imu_link
    static_imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'robot1/imu_link/imu_sensor'
        ]
    )

    # Add Object Detection Node
    object_detection_node = Node(
        package='butterflybot',
        executable='object_detection_node',  # This stays the same
        name='object_detection_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output='screen'
    )

    # Joint controller in a new terminal window
    joint_controller_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c', 
            'ros2 run butterflybot joint_controller; exec bash'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', path_to_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Cartographer SLAM Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'tf_buffer_qos_durability': 'transient_local'}
        ],
        arguments=[
            '-configuration_directory', path_to_config,
            '-configuration_basename', 'butterflybot_2d.lua'
        ],
        remappings=[
           # ('odom', '/odom'),
            ('scan', '/lidar_scan'),
            ('/imu', '/imu/data'),
        ]
    )

    # Occupancy Grid Node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.01},
            {'publish_period_sec': 1.0}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        set_gz_sim_resource_path,
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        rviz_node,
        transform_broadcaster_node,
        static_transform_node,
        static_lidar_transform,
        static_imu_transform,
        object_detection_node,
        joint_controller_node,
        joint_state_publisher_gui_node,
        cartographer_node,
        cartographer_occupancy_grid_node
    ])