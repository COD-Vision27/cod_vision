from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    default_config = str(Path(get_package_share_directory('sp_vision_ros'))/'configs/standard3.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='sp_vision_ros', executable='sp_sim_auto_aim',
             parameters=[{'config': LaunchConfiguration('config')}], output='screen'),
    ])
