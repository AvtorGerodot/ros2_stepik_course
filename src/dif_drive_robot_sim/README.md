# dif_drive_robot_sim
## Применение
Данный пакет описывает симуляцию робота на дифференциальной платформе с лидаром в gazebo (harmonic), а также отображает его положение и данные с лидара в rviz2. Управлять роботом можно через teleop_twist_keyboard
## Пакет включает в себя:  
- sdf модель комнаты 
- urdf модель робота на дифференциальной платформе
- yaml файлы для описания топиков для создания моста, а также для создания дифференциального контроллера
- launch файл для запуска gazebo симуляции, rviz2, настройки поста, подключения контроллеров

## Запуск
После сборки проекта и выполнения source  
```
ros2 launch dif_drive_robot_sim gz_two_wheel_robot.launch.py 
```  
Запуск teleop_twist_keyboard с изменённым топиком и типом TwistStamped:  
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard     --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```