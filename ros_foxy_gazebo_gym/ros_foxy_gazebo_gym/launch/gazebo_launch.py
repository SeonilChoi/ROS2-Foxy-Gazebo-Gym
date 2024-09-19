import os
import xacro

from billiard import Process

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.chdir(path)

def generate_launch_description(world_file, urdf_file, robot_name, spawn_pose):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    this_pkg_path     = os.path.join('ros_foxy_gazebo_gym')
    gazebo_world_path = os.path.join(this_pkg_path, 'worlds', world_file)
    xacro_file_path   = os.path.join(this_pkg_path, 'urdf', robot_name, urdf_file)
    gazebo_pkg_path   = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    with open(xacro_file_path, 'r') as f:
        doc = xacro.parse(f)
        xacro.process_doc(doc)
        robot_description = doc.toxml()

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_path, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world':gazebo_world_path, 'pause':'false', 'verbose':'false'}.items()
    )
    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_path, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time, 'robot_description':robot_description}],
        arguments=[xacro_file_path]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', robot_name,
                   '-x', spawn_pose['x'],
                   '-y', spawn_pose['y'],
                   '-z', spawn_pose['z'],
                   '-R', spawn_pose['R'],
                   '-P', spawn_pose['P'],
                   '-Y', spawn_pose['Y']],
        output='screen'
    )
           
    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        spawn_entity]
    )

def startLaunchServiceProcess(launchDesc):
    launchService = LaunchService()
    launchService.include_launch_description(launchDesc)

    process = Process(target=launchService.run)
    process.daemon = True
    process.start()
    return process
