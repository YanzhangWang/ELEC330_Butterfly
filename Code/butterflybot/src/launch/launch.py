import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file_name = 'butterflybot.urdf'
    rviz_config = 'butterfly.rviz'

    path_to_urdf = os.path.join(
        get_package_share_directory('butterflybot'),
        'urdf',
        urdf_file_name)

    path_to_world = os.path.join(
        get_package_share_directory('butterflybot'),
        'urdf',
        'env_ws.sdf')

    path_to_rviz = os.path.join(
        get_package_share_directory('butterflybot'),
        'rviz',
        rviz_config)

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
            'robot_description': ParameterValue(Command(['xacro ', path_to_urdf]), value_type=str),
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
            "-file", path_to_urdf,
            "-x", "-1", "-y", "0", "-z", "4"
        ],
        output="screen",
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/boxes_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/Right_joint_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/Left_joint_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/pp_Right_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/pp_Left_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/lidar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera1_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        set_gz_sim_resource_path,
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        rviz_node,
    ])
