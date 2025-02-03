import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the map_divider config
    map_divider_config_path = launch.substitutions.LaunchConfiguration(
        'map_divider_config',
        default=os.path.join(
            get_package_share_directory('map_divider'),
                'config',
                'config_map_divider.yaml'
        )
    )

    waypoint_navigator_node = Node(
        package='map_divider',
        executable='map_divider_node',
        name='map_divider_node',
        output='screen',
        parameters=[map_divider_config_path]
    )
    
    ld.add_action(waypoint_navigator_node)

    return ld