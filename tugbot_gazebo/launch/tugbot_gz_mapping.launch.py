from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import tempfile

def generate_launch_description():
    world_file_original = os.path.join(
        get_package_share_directory('tugbot_gazebo'),
        'worlds',
        'tugbot_depot.sdf'
    )
    
    # Get the package share directory for resource path
    tugbot_description_share = get_package_share_directory('tugbot_description')
    
    # Get the absolute path to the model.sdf file
    model_sdf_path_absolute = os.path.join(
        tugbot_description_share,
        'urdf',
        'model.sdf'
    )
    
    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find the meshes and models
    # GZ_SIM_RESOURCE_PATH is a colon-separated list of directories where Gazebo looks for resources
    # It's used to resolve relative paths in model:// and file:// URIs
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gz_resource_path:
        # If already set, append our package share directory (avoid duplicates)
        if tugbot_description_share not in gz_resource_path:
            gz_resource_path = f'{gz_resource_path}:{tugbot_description_share}'
    else:
        # If not set, use only our package share directory
        gz_resource_path = tugbot_description_share
    
    # Use absolute file path for the model (file:// URIs require absolute paths)
    # The path points to tugbot_description/urdf/model.sdf in the package share directory
    model_sdf_path = model_sdf_path_absolute
    
    # Read the world file and replace the model URI
    with open(world_file_original, 'r') as f:
        world_content = f.read()
    
    # Replace the placeholder or Fuel URI with the local model path
    world_content = world_content.replace(
        'file://MODEL_SDF_PATH',
        f'file://{model_sdf_path}'
    )
    world_content = world_content.replace(
        'https://fuel.ignitionrobotics.org/1.0/MovAi/models/Tugbot',
        f'file://{model_sdf_path}'
    )
    
    # Write to a temporary file
    temp_world_file = os.path.join(tempfile.gettempdir(), 'tugbot_depot_modified.sdf')
    with open(temp_world_file, 'w') as f:
        f.write(world_content)
    
    world_file_path = temp_world_file

    ros_gz_bridge_launch_path = PathJoinSubstitution([
        get_package_share_directory('tugbot_gazebo'),
        'launch',
        'ros_gz_bridge.launch.py'
    ])
    
    robot_description_launch_path = PathJoinSubstitution([
        get_package_share_directory('tugbot_description'),
        'launch',
        'tugbot_description.launch.py'
    ])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            gz_resource_path
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', f'{world_file_path}')]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_gz_bridge_launch_path),
            launch_arguments=[
                ('enable_slam', 'true'),
                ('enable_rviz', 'true')]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch_path)
        )
    ])

