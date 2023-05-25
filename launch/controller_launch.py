import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    ld = LaunchDescription()
    param_file = os.path.join(get_package_share_directory('pdwc'), 'params', 'params.yaml')

    controller_node = Node(
        package='pdwc',
        executable='controller',
        emulate_tty=True,
        output="screen",
        remappings=[
            ('cmd_vel', '/robot_0/cmd_vel'),
            # ('global_pose', '/robot_0/global_pose'),
            # ('cmd_vel', 'itav_agv/cmd_vel'),
            ('odom', 'robot_0/odom'),
        ],
        parameters=[
            {'rate': 10},
            {'footprint': "[[1.7, 0.6], [1.7, -0.6], [-0.6, -0.6], [-0.6, 0.6]]"},
            {'max_speed': 0.65},
            {'min_speed': -0.65},
            {'max_yawrate': 0.707},
            {'max_accel': 6.0},
            # {'max_dyawrate': 600.0},
            # {'max_dyawrate': 100.0},
            {'max_dyawrate': 300.0},
            {'v_reso': 0.3},
            {'yawrate_reso': 0.1},
            {'dt': 0.1},
            {'predict_time': 3.0},
            {'to_goal_cost_gain': 2.0},
            {'speed_cost_gain': 0.750},
            {'obstacle_cost_gain': 2.50},
            {'goal_tolerance': 0.250},
            {'reverse_penality': 50.0},
            {'robot_radius': 0.5},
            {'obstacle_radius': 0.5},
            {'obstacle_margin': 1.250},
            {'collision_threshold': 0.005},
        ],
        
    )
    transform_node = Node(
        package='pdwc',
        executable='transform.py',
        name='transform_node',
        emulate_tty=True,
        output="screen",
        remappings=[
            ('odom', 'robot_0/odom'),
            # ('odom', 'itav_agv/odom'),
            # ('global_pose', 'itav_agv/global_pose'),
        ]
    
    )
    ld.add_action(controller_node)
    ld.add_action(transform_node)
    # create and return launch description object
    return ld