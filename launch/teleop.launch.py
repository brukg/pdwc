import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
 
    teleop_node = Node(
        package='itav_agv_controller',
        executable='teleop',
        name='teleop_node',
        emulate_tty=True,
        output="screen",
        remappings=[
            ('cmd_vel', 'itav_agv/cmd_vel'),
        ]
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            teleop_node
        ]
    )