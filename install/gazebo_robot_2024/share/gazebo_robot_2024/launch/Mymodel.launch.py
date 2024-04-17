import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro
import yaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    conveyorbelt_gazebo = os.path.join(
        get_package_share_directory('gazebo_robot_2024'),
        'worlds',
        'mymodel.world')
    #print(conveyorbelt_gazebo)
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': conveyorbelt_gazebo}.items(),
             )
    
    under = Node(
            package = 'controll_robot',
            executable = 'under',
            output = 'screen'
        )
    package_dir = get_package_share_directory("gazebo_robot_2024")
    rviz = os.path.join(package_dir, "rviz" , "lidar.rviz")
    
    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz]
            ),
        #under 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'),
    ])