import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('ee3033_search')) / 'config' / 'search.yaml')
    params = [config]


    target = os.environ.get('EE3033_TARGET', '').strip().lower()
    if target:
        params.append({'target_class': target})
    return LaunchDescription([
        Node(package='ee3033_search', executable='search', name='ee3033_search',
             output='screen', parameters=params),
    ])
