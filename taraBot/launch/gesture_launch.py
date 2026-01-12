from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    gesture_recognizer = Node(
        package='taraBot',
        executable='gesture_node',
        name='gesture_recognizer',
        output='screen',
        parameters=[{
            'language': '/home/francis/IRS_ws/src/taraBot/gestures.json'
        }]
    )

    turtlebot_controller = Node(
        package='taraBot',
        executable='turtlebot_gesture_controller',
        name='gesture_controller',
        output='screen'
    )

    return LaunchDescription([
        gesture_recognizer,
        turtlebot_controller
    ])
