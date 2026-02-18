# Описание пакетов
## lidar_stop


## my_fake_lidar


## my_path_publisher


## my_robot_project
не является пакетом, а скорее доп. папка для хранения описания .sdf мира в gazebo

## my_two_wheel_robot
Основной пакет для связи gazebo, ros2, отображения робота в rviz2

## twist_converter
Пакет для конвертации сообщений типа Twist из топика '/cmd_vel_unstamped' в тип TwistStamped и публикации в '/diff_drive_controller/cmd_vel'  
Запуск конвертера:  
```ros2 run twist_converter twist_to_twist_stamped ```  
Запуск teleop_twist_keyboard:  
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_unstamped ```  

## view_robot
Пробный пакет для описания объектов в urdf и отображения их в rviz2  
В проекте **не** используется 
