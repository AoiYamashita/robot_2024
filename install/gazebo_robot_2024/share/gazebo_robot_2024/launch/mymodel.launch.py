import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_dir = get_package_share_directory("gazebo_robot_2024")
    urdf = os.path.join(package_dir, "urdf" , "first_robot.urdf")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
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
    
    controll = Node(
            package = 'gazebo_robot_2024',
            executable = 'controll',
            output = 'screen'
        )
    yaslam = Node(
        package="robot_2024",
        executable="yaslam",
        output="screen"
    )
    package_dir = get_package_share_directory("gazebo_robot_2024")
    rviz = os.path.join(package_dir, "rviz" , "lidar.rviz")

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf],
            ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf],
            ),
        
        # gazebo settings
        gazebo,

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz]
            ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="urdf_spawner",
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=["-x" ,"-0.5","-y" ,"-1","-z" ,"1.0","-topic", "/robot_description", "-entity", "first_robot"],
            ),
        controll,
        yaslam,
    ])