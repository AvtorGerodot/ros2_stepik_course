import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_share = get_package_share_directory('dif_drive_robot_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'dif_robot_description', 'dif_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_robot.rviz')
    controller_params_file = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    # пути для gazebo и bridge
    world_path = os.path.join(pkg_share, 'worlds', 'room.sdf')
    gz_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_bridge_params_file = os.path.join(pkg_share, 'config', 'gz_bridge_param.yaml')

    # пути для slam_toolbox
    slam_launch_file = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_param.yaml')


    # Генерируем Substitution для xacro -> urdf; Явно указываем, что это строка; Готовим словарь
    robot_description_substitution = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    robot_description_param = {'robot_description': robot_description}



    # запуск симулятора gazebo
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file),
         launch_arguments={
            'gz_args': f'-r {world_path}', # флаг -r необходим для запуска симуляции
            'on_exit_shutdown': 'True'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # '-entity', 'dif_robot',
            '-name', 'dif_robot',
            '-topic', 'robot_description',
            # или: '-file', путь_к_urdf, если не через параметр
            '-z', '0.04',
        ],
        output='screen'
    )

    # запуск моста gazebo - ros2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        parameters=[{'config_file': gz_bridge_params_file}],
        output='screen',
    )

    # Узел robot_state_publisher 
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param,
                    {'use_sim_time': True}]  # <-- передаём dict
    )

    # Спавним контроллеры
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',  '--param-file', controller_params_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # (Опционально) узел joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # slam_toolbox
    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
         launch_arguments={
            'slam_params_file': slam_params_file,
        }.items()
    )

    # Собираем всё в LaunchDescription
    return LaunchDescription([
        start_gz_sim,                   # 1
        gz_bridge,                      # 2

        rsp_node,                       # 3
        joint_state_spawner,            # 4
        diff_drive_spawner,             # 5
        joint_state_publisher_node,     # 6

        rviz_node,                      # 7
        spawn_robot,                    # 8
        start_slam,                     # 9
    ])
