#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class LidarStopNode(Node):
    def __init__(self):
        super().__init__('lidar_stop_node')
        # Порог остановки, м
        self.stop_distance = 0.5

        # Подписка на /lidar (LaserScan) - из ROS2 через мост
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.laser_callback,
            10
        )

        # Публикация Twist в /cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,  # geometry_msgs/msg/Twist
            '/cmd_vel',
            10
        )

        # Таймер (10 Гц)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.obstacle_detected = False

    def laser_callback(self, msg: LaserScan):
        # Найдём минимальную дистанцию по всему скану
        min_range = min(msg.ranges) if msg.ranges else 9999.0
        self.obstacle_detected = (min_range < self.stop_distance)

    def timer_callback(self):
        # Простейшая логика: если препятствие, публикуем 0 скорость, иначе двигаемся вперёд
        twist = Twist()
        if self.obstacle_detected:
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.05

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
