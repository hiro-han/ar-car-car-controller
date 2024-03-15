import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = launch.substitutions.LaunchConfiguration(
        'params', default=[launch.substitutions.ThisLaunchFileDir(), '/../config/params.yaml']


    return LaunchDescription([
        
        launch.actions.LogInfo(
            msg="Launch AR Car."
        ),
        
        # Node for AR Car Controller
        Node(
            package         = 'ar-car-controller',
            namespace       = 'ar-car-controller',
            executable      = 'ar-car-controller',
            name            = 'ar-car-controller',
            parameters      = [params_file]
            ),

        # Node for USB Camera
        Node(
            package         = 'v4l2_camera',
            namespace       = 'v4l2_camera',
            executable      = 'v4l2_camera_node',
            name            = 'v4l2_camera_node',
            parameters      = [params_file]
            ),
    ])