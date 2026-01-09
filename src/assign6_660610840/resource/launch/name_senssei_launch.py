from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    lidar_dir = get_package_share_directory('sllidar_ros2')
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'sllidar_a3_launch.py')
        ),
    )
    
    lidar_listener = Node(
        package="assign6_660610840",
        executable="lidar_listener",
        name="lidar_listener",
        output="screen",
    )

    ld.add_action(lidar_listener)
    ld.add_action(lidar_launch)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()
