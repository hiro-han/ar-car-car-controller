import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
\
    return LaunchDescription([
        
        launch.actions.LogInfo(
            msg="Launch AR Car."
        ),
        
        # Node for AR Car Controller
        Node(
            package         = 'ar_car_controller',
            namespace       = 'ar_car_controller',
            executable      = 'test_serial',
            name            = 'test_serial',
            parameters      = [{'device': "/dev/ttyACM0", 'speed_limit_rate': 0.5}],
            ros_arguments   = ['--log-level', 'debug'],
        )
    ])
