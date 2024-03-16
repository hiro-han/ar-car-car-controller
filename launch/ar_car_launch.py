import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = launch.substitutions.LaunchConfiguration(
        'params', default=[launch.substitutions.ThisLaunchFileDir(), '/params.yaml'])


    return LaunchDescription([
        
        launch.actions.LogInfo(
            msg="Launch AR Car."
        ),
        
        # Node for AR Car Controller
        Node(
            package         = 'ar_car_controller',
            namespace       = 'ar_car_controller',
            executable      = 'ar_car_controller',
            name            = 'ar_car_controller',
            parameters      = [params_file]
            ),

        # Node for USB Camera
        Node(
            package         = 'v4l2_camera',
            namespace       = 'v4l2_camera',
            executable      = 'v4l2_camera_node',
            name            = 'v4l2_camera_node',
            remappings=[
                ('/v4l2_camera/image_raw/compressed', '/image_raw/compressed'),
                ]
            ),
    ])