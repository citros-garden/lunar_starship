import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ld = LaunchDescription()
    lunar_starship_config = os.path.join(
        get_package_share_directory('lunar_starship'),
        'config',
        'params.yaml'
        )
    lunar_starship=Node(
        package = 'lunar_starship',
        name = 'lunar_starship',
        executable = 'lunar_starship',
        parameters = [lunar_starship_config]
    )

    bridge_dir = get_package_share_directory('rosbridge_server')
    bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 
    ld.add_action(bridge_launch)
    ld.add_action(lunar_starship)
    return ld