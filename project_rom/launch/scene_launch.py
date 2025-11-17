# scene_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, LogInfo, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def launch_setup(context, *args, **kwargs):
    path_file = LaunchConfiguration('path_file').perform(context)

    waypoints_node = Node(
        package='project_rom',
        executable='waypoints_node',
        name='waypoints_node',
        output='screen',
        parameters=[{'path_file': path_file}],
    )

    pure_pursuit_node = Node(
        package='project_rom',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
    )

    obstacle_avoider_node = Node(
        package='project_rom',
        executable='obstacle_avoider_node',
        name='obstacle_avoider_node',
        output='screen'
    )

    return [
        waypoints_node,
        obstacle_avoider_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=waypoints_node,
                on_start=[
                    LogInfo(msg='📍 Waypoints listos, lanzando el controlador...'),
                    pure_pursuit_node
                ]
            )
        )
    ]

def generate_launch_description():
    # Configuraciones y rutas
    scene_dir = LaunchConfiguration('scene_dir')
    path_file = LaunchConfiguration('path_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_dir = get_package_share_directory('project_rom')
    rviz_launch_file = os.path.join(pkg_dir, 'launch', 'rviz2_launch.py')
    default_scene = os.path.join(pkg_dir, 'scenes', 'sonar_scene.ttt')

    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf = os.path.join(pkg_dir, 'urdf', urdf_file_name)

    # Declaración de argumentos
    declare_scene_dir = DeclareLaunchArgument(
        'scene_dir',
        default_value=default_scene,
        description='Ruta al archivo .ttt'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar reloj simulado'
    )

    declare_path_file_cmd = DeclareLaunchArgument(
        'path_file',
        default_value=os.path.join(pkg_dir, 'config', 'path.yaml'),
        description='Fichero con los puntos del path'
    )

    return LaunchDescription([
        declare_scene_dir,
        declare_use_sim_time,
        declare_path_file_cmd,

        LogInfo(msg=['🟢 Lanzando CoppeliaSim con: ', scene_dir]),

        ExecuteProcess(
            cmd=[['coppeliaSim.sh -f ', scene_dir, ' -s 0']],
            shell=True
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file)
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        OpaqueFunction(function=launch_setup)
    ])
