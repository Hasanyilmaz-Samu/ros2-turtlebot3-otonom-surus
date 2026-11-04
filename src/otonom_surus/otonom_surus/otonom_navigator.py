#!/usr/bin/env python3
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
import yaml


class OtonomNavigator(Node):
    def __init__(self):
        super().__init__('otonom_navigator')

        # Parametreler (hem string '1' hem tamsayı 1 kabul eder)
        descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('senaryo', 1, descriptor)
        self.declare_parameter('goal_tolerance', 0.40)  # Hedefe kaç metre kala 'vardı' sayılsın

        senaryo_no = str(self.get_parameter('senaryo').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        pkg_dir = get_package_share_directory('otonom_surus')
        config_path = os.path.join(pkg_dir, 'config', 'scenarios.yaml')

        with open(config_path, 'r') as f:
            scenarios = yaml.safe_load(f)['senaryolar']

        if senaryo_no not in scenarios:
            self.get_logger().error(f"Gecersiz senaryo: {senaryo_no}. Mevcut: {list(scenarios.keys())}")
            sys.exit(1)

        self.scenario = scenarios[senaryo_no]
        self.senaryo_no = senaryo_no

        # Tekil Hedef (Ara hedef yok, dogrudan final hedefe otonom surus)
        goal = self.scenario['hedef']
        self.target_mx = float(goal['x'])
        self.target_my = float(goal['y'])

        self.get_logger().info(
            f"=== Otonom Surus Baslatildi (Senaryo {senaryo_no}: {self.scenario['ad']}) ===\n"
            f"Hedef Koordinati: ({self.target_mx:.2f}, {self.target_my:.2f})"
        )

        # Metrik degiskenleri
        self.current_x = None
        self.current_y = None
        self.prev_x = None
        self.prev_y = None
        self.start_time = None
        self.end_time = None
        self.total_distance = 0.0
        self.min_obstacle_dist = float('inf')
        self.state = 'WAIT_SIM'  # WAIT_SIM -> PREPARE -> NAVIGATING -> DONE
        self.last_goal_publish_time = 0.0
        self.is_paused = False

        # Yayinci ve Abonelikler
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/acil_durdur', self.emergency_callback, 10)

        # Kontrol Dongusu (5 Hz)
        self.timer = self.create_timer(0.2, self.control_loop)
        self.last_log_time = 0.0

    def emergency_callback(self, msg: Bool):
        if msg.data:  # True = Acil Durdur
            if not self.is_paused:
                self.is_paused = True
                if self.cancel_client.service_is_ready():
                    self.cancel_client.call_async(CancelGoal.Request())
                self.get_logger().warn("[ACIL DURDURMA] Manuel kontrol (x) ile otonom surus durduruldu!")
        else:  # False = Devam et (r)
            if self.is_paused:
                self.is_paused = False
                self.get_logger().info("[DEVAM] Otonom surus yeniden baslatildi, hedefe devam ediliyor...")
                self.send_goal()

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        if self.state == 'NAVIGATING':
            if self.prev_x is not None:
                step = math.hypot(pos.x - self.prev_x, pos.y - self.prev_y)
                if step < 0.5:  # Sicramalari engelle
                    self.total_distance += step
            self.prev_x = pos.x
            self.prev_y = pos.y

    def scan_callback(self, msg: LaserScan):
        if self.state == 'NAVIGATING':
            # 0.20 m altindaki degerler robotun kendi govdesidir, filtrele
            valid = [r for r in msg.ranges if max(0.20, msg.range_min) < r < msg.range_max]
            if valid:
                m = min(valid)
                if m < self.min_obstacle_dist:
                    self.min_obstacle_dist = m

    def send_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = self.target_mx
        goal_msg.pose.position.y = self.target_my
        goal_msg.pose.position.z = 0.0

        cur_x = self.current_x if self.current_x is not None else -3.0
        cur_y = self.current_y if self.current_y is not None else -3.0
        target_yaw = math.atan2(self.target_my - cur_y, self.target_mx - cur_x)
        goal_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(
            f"--> Hedef /goal_pose yayinlandi: ({self.target_mx:.2f}, {self.target_my:.2f})"
        )

    def control_loop(self):
        now = time.time()

        if self.is_paused:
            return

        if self.state == 'WAIT_SIM':
            if self.current_x is not None:
                self.get_logger().info("Odometri alindi. Nav2 baglantisi kontrol ediliyor...")
                self.state = 'PREPARE'

        elif self.state == 'PREPARE':
            sub_count = self.goal_pub.get_subscription_count()
            if sub_count > 0:
                self.get_logger().info(f"Nav2 /goal_pose baglantisi aktif ({sub_count} abone). Hedef gonderiliyor...")
                self.start_time = now
                self.last_goal_publish_time = now
                self.prev_x = self.current_x
                self.prev_y = self.current_y
                self.send_goal()
                self.state = 'NAVIGATING'
            else:
                if now - self.last_log_time >= 2.0:
                    self.get_logger().info("Nav2 /goal_pose abonesi bekleniyor...")
                    self.last_log_time = now

        elif self.state == 'NAVIGATING':
            # Eger robot henuz hareket etmediyse her 2 saniyede bir tekrar dene
            if self.total_distance < 0.05 and (now - self.last_goal_publish_time >= 2.0):
                self.get_logger().info("Robot henuz hareket etmedi, hedef tekrar gonderiliyor...")
                self.send_goal()
                self.last_goal_publish_time = now

            rem_dist = math.hypot(self.target_mx - self.current_x, self.target_my - self.current_y)

            if now - self.last_log_time >= 2.0:
                self.get_logger().info(
                    f"Hedefe Kalan: {rem_dist:.2f} m | Toplam Yol: {self.total_distance:.2f} m | "
                    f"En Yakin Engel: {self.min_obstacle_dist:.2f} m"
                )
                self.last_log_time = now

            # Hedefe varildi mi?
            if rem_dist <= self.goal_tolerance:
                self.end_time = now
                self.finish_mission(success=True)

            # Zaman asimi (4 dakika)
            elif (now - self.start_time) > 240.0:
                self.end_time = now
                self.finish_mission(success=False)

    def finish_mission(self, success):
        self.state = 'DONE'
        duration = self.end_time - self.start_time
        avg_speed = (self.total_distance / duration) if duration > 0 else 0.0

        report = f"""
============================================================
              DENEYSEL SONUC RAPORU
============================================================
Senaryo            : {self.senaryo_no} ({self.scenario['ad']})
Hedef Durumu       : {'BASARILI (Hedefe Ulasildi)' if success else 'BASARISIZ (Zaman Asimi)'}
Toplam Gecen Sure  : {duration:.2f} saniye
Kat Edilen Mesafe  : {self.total_distance:.2f} metre
Ortalama Hiz       : {avg_speed:.2f} m/s
En Yakin Engel     : {self.min_obstacle_dist:.2f} metre
============================================================
"""
        self.get_logger().info(report)

        log_dir = os.path.expanduser('~/Masaüstü/ros2_deneme')
        log_file = os.path.join(log_dir, 'deney_sonuclari.txt')
        with open(log_file, 'a') as f:
            f.write(report)
        self.get_logger().info(f"Sonuclar kaydedildi: {log_file}")


def main(args=None):
    rclpy.init(args=args)
    node = OtonomNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
