import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pcdtogrid'), 'config', 'params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        Node(
            package='pcdtogrid',
            executable='ptgm',
            name='ptgm',
            parameters=[config, {'use_sim_time': use_sim_time}]
        )
    ])
