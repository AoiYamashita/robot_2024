import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    webcontroll = launch_ros.actions.Node(
            package = 'robot_2024',
            executable = 'webControll',
            output = 'screen',
            )
    serial = launch_ros.actions.Node(
            package = 'robot_2024',
            executable = 'serial',
            output = 'screen',
            )
    web = launch_ros.actions.Node(
            package = 'rosbridge_server',
            executable = 'rosbridge_websocket.py',
            parameters=[{'port': 9092}],
            output = 'screen',
            )
    return launch.LaunchDescription([web,webcontroll,serial])
