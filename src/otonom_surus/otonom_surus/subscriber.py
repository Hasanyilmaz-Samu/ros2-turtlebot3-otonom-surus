#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class RobotSubscriber(Node):
    def __init__(self):
        super().__init__('robot_subscriber')

        # Durum degiskenleri
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        self.min_obstacle_dist = float('inf')
        self.obstacle_angle_deg = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.active_mode = "BEKLEMEDE"
        self.emergency_stopped = False

        # Abonelikler
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Bool, '/acil_durdur', self.emergency_callback, 10)

        # 2 Hz Konsol Yazdirma
        self.timer = self.create_timer(0.5, self.display_status)
        self.get_logger().info("Robot Canli Telemetri Abonesi (Subscriber) Baslatildi.")

    def emergency_callback(self, msg: Bool):
        self.emergency_stopped = msg.data

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos_x = p.x
        self.pos_y = p.y

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.lin_speed = msg.twist.twist.linear.x
        self.ang_speed = msg.twist.twist.angular.z

    def scan_callback(self, msg: LaserScan):
        min_d = float('inf')
        min_idx = 0
        for idx, r in enumerate(msg.ranges):
            if max(0.20, msg.range_min) < r < msg.range_max:
                if r < min_d:
                    min_d = r
                    min_idx = idx

        self.min_obstacle_dist = min_d
        if min_d != float('inf'):
            angle = msg.angle_min + min_idx * msg.angle_increment
            self.obstacle_angle_deg = math.degrees(angle)

    def cmd_callback(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.active_mode = "HAREKET HALINDE (SURUS)"
        else:
            self.active_mode = "DURUYOR"

    def display_status(self):
        # Yon tespiti (engele gore)
        if self.min_obstacle_dist < float('inf'):
            deg = self.obstacle_angle_deg
            if -30 <= deg <= 30:
                direction = "Tam Ön"
            elif 30 < deg <= 90:
                direction = "Ön-Sol"
            elif 90 < deg <= 150:
                direction = "Arka-Sol"
            elif -90 <= deg < -30:
                direction = "Ön-Sağ"
            elif -150 <= deg < -90:
                direction = "Arka-Sağ"
            else:
                direction = "Tam Arka"
            obstacle_str = f"{self.min_obstacle_dist:.2f} m ({direction}, {deg:+.1f}°)"
        else:
            obstacle_str = "Menzilde engel yok"

        # Uyari durumu
        if self.emergency_stopped:
            safety_status = "ACIL DURDURMA DEVREDE ([x] ile kilitlendi)"
            mode_display = "DURDURULDU (Acil Durdurma)"
        elif self.min_obstacle_dist < 0.35:
            safety_status = "DIKKAT: ENGEL COK YAKIN!"
            mode_display = self.active_mode
        else:
            safety_status = "GUVENLI"
            mode_display = self.active_mode

        output = f"""
============================================================
              ROBOT CANLI TELEMETRI (SUBSCRIBER)
============================================================
Konum (Odom)      : X: {self.pos_x:+.2f} m  |  Y: {self.pos_y:+.2f} m  |  Yön: {self.yaw:+.1f}°
Gercek Hız        : Lineer: {self.lin_speed:+.2f} m/s  |  Açısal: {self.ang_speed:+.2f} rad/s
Komut Hızı (cmd)  : Lineer: {self.cmd_linear:+.2f} m/s  |  Açısal: {self.cmd_angular:+.2f} rad/s
En Yakın Engel    : {obstacle_str}
Güvenlik Durumu   : {safety_status}
Robot Durumu      : {mode_display}
============================================================
"""
        # Ekrani temizlemeden temizce yazdir
        print(output)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
