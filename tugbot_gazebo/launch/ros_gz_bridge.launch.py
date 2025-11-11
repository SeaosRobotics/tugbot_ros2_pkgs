from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Load URDF for robot_state_publisher
    urdf_path = os.path.join(
        get_package_share_directory('tugbot_description'),
        'urdf',
        'model.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('enable_slam', default_value='true',
                              description='Enable SLAM mapping'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                              description='Enable RViz visualization'),

        # Robot State Publisher - publishes TF tree from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            arguments=[
                '/model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/model/tugbot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/model/tugbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                '/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/world_demo/model/tugbot/link/scan_back/sensor/scan_back/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            remappings=[
                ('/model/tugbot/tf', '/tf'),
                ('/model/tugbot/odometry', '/odom'),
            ],
            output='screen',
            parameters=[
                {"use_sim_time": use_sim_time}
            ],
        ),

        # Convert pointcloud to laserscan for SLAM
        # Use all layers of the omni-directional scan, but filter out ground points
        # min_height filters out points hitting the ground (below this height)
        # max_height allows all upper layers to be included
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                {'target_frame': 'base_link'},
                {'transform_tolerance': 0.5},
                {'min_height': 0.15},  # Filter out ground points (below 15cm from base_link)
                {'max_height': 2.0},   # Include all layers up to 2m height
                {'angle_min': -3.14159},
                {'angle_max': 3.14159},
                {'angle_increment': 0.00872665},
                {'scan_time': 0.1},
                {'range_min': 0.3},
                {'range_max': 30.0},
                {'use_inf': True},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('cloud_in', '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points'),
                ('scan', '/scan')
            ],
            condition=IfCondition(LaunchConfiguration('enable_slam')),
        ),

        # SLAM Toolbox node (lifecycle node)
        LifecycleNode(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[
                get_package_share_directory('tugbot_slam') + '/config/mapper_params_offline.yaml',
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/scan', '/scan'),
            ],
            condition=IfCondition(LaunchConfiguration('enable_slam')),
        ),

        # Configure SLAM Toolbox lifecycle (wait 3 seconds for node to start)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/slam_toolbox/change_state', 
                         'lifecycle_msgs/srv/ChangeState', 
                         '{transition: {id: 1, label: configure}}'],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('enable_slam')),
                )
            ],
            condition=IfCondition(LaunchConfiguration('enable_slam')),
        ),
        # Activate SLAM Toolbox lifecycle (wait 6 seconds total to ensure configure completes)
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/slam_toolbox/change_state', 
                         'lifecycle_msgs/srv/ChangeState', 
                         '{transition: {id: 3, label: activate}}'],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('enable_slam')),
                )
            ],
            condition=IfCondition(LaunchConfiguration('enable_slam')),
        ),

        # Static transform from map to odom (initial identity transform)
        # This ensures the TF tree is complete before SLAM initializes
        # SLAM Toolbox will override this transform once it starts publishing
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(LaunchConfiguration('enable_slam')),
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                get_package_share_directory('tugbot_slam') + '/rviz/default.rviz'
            ],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
        ),
    ])

