import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control') # This might be handled within the URDF or SDF now

    # Specify the name of the package and path to URDF file within the package
    pkg_name = 'silver'
    # OPTION 1: For a URDF file
    file_subpath_urdf = 'description/silver.urdf'
    urdf_file_path = os.path.join(get_package_share_directory(pkg_name), file_subpath_urdf)

    # Read the URDF file content
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # Configure the node with the URDF content
    # The robot_description parameter expects the content of the URDF/SDF file as a string
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true'),
        # DeclareLaunchArgument( # You might still need this if other parts of your system use it
        # 'use_ros2_control',
        # default_value='true',
        # description='Use ros2_control if true'),
        node_robot_state_publisher
    ])