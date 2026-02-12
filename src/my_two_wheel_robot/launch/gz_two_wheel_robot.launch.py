import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('my_two_wheel_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_two_wheel_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_lidar.rviz')
    controller_params_file = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    gz_bridge_params_file = os.path.join(pkg_share, 'config', 'gz_bridge_param.yaml')




    # 1) Генерируем Substitution для xacro -> urdf
    robot_description_substitution = Command(['xacro ', xacro_file])
    # 2) Явно указываем, что это строка
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    # 3) Готовим словарь
    robot_description_param = {'robot_description': robot_description}


    # --- запуск моста gazebo - ros2 ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        parameters=[{'config_file': gz_bridge_params_file}],
        output='screen',
    )

    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        arguments=[
            '0', '0', '0',          # x y z
            '0', '0', '0',          # roll pitch yaw
            'lidar_link',           # parent frame
            'my_robot/lidar_link/lidar_sensor'  # child frame
        ],
        output='screen'
    )

    # --- Узел robot_state_publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param,
                    {'use_sim_time': True}]  # <-- передаём dict
    )

    # --- Узел ROS2 Control (controller_manager) ---
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description_param,  # словарь (robot_description)
            controller_params_file,    # путь к YAML
            {'use_sim_time': True}
        ]
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
    # Для дифф. робота обычно не нужен, но если хотите видеть слайдеры, оставьте
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Собираем всё в LaunchDescription
    return LaunchDescription([
        gz_bridge,
        static_lidar_tf,
        rsp_node,
        ros2_control_node,
        joint_state_spawner,
        diff_drive_spawner,
        joint_state_publisher_node,
        rviz_node,
    ])
