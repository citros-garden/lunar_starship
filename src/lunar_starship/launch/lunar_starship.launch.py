import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

class bcolors:
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

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

    sys_shut_down = RegisterEventHandler(OnProcessExit(
    target_action=lunar_starship,
    on_exit=[
                LogInfo(msg=(f'{bcolors.OKGREEN}The Scenario has ended!{bcolors.ENDC}')),
                EmitEvent(event=Shutdown(
                    reason='Finished'))
            ]		
    ))

    bridge_dir = get_package_share_directory('rosbridge_server')
    bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 
    ld = LaunchDescription ([bridge_launch, lunar_starship, sys_shut_down])
    return ld