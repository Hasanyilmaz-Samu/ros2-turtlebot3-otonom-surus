#!/usr/bin/env python3
import math
import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
import yaml


class OtonomKesif(Node):
    def __init__(self):
        super().__init__('otonom_kesif')

        # Parametreler
        descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('senaryo', 1, descriptor)

        senaryo_no = str(self.get_parameter('senaryo').value)
        pkg_dir = get_package_share_directory('otonom_surus')
        config_path = os.path.join(pkg_dir, 'config', 'scenarios.yaml')

        with open(config_path, 'r') as f:
            scenarios = yaml.safe_load(f)['senaryolar']

        self.scenario = scenarios.get(senaryo_no, {'ad': 'bilinmeyen'})
        self.senaryo_no = senaryo_no

        self.get_logger().info(
            f"=== Otonom Sinir Kesfi (Frontier Exploration) Baslatildi ===\n"
            f"Senaryo: {senaryo_no} ({self.scenario['ad']})\n"
            f"Robot tum ortami otomatik gezip haritayi cikaracak ve kaydedecektir."
        )

        # Durum degiskenleri
        self.map_msg = None
        self.current_x = None
        self.current_y = None
        self.prev_x = None
        self.prev_y = None
        self.start_time = None
        self.total_distance = 0.0
        self.min_obstacle_dist = float('inf')
        self.target_x = None
        self.target_y = None
        self.state = 'WAIT_MAP'  # WAIT_MAP -> SEARCH_FRONTIER -> EXPLORING -> FINISHED
        self.target_sent_time = 0.0
        self.last_log_time = 0.0
        self.last_progress_x = None
        self.last_progress_y = None
        self.last_progress_time = 0.0
        self.blacklist = []  # Ulasilamayan veya takilinan sinirlar
        self.visited_goals = []  # [(x, y, timestamp)] son ziyaret edilen hedefler
        self.is_paused = False

        # Yayinci ve Abonelikler
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/acil_durdur', self.emergency_callback, 10)

        # 2 Hz Dongu
        self.timer = self.create_timer(0.5, self.exploration_loop)

    def emergency_callback(self, msg: Bool):
        if msg.data:  # True = Acil durdur
            if not self.is_paused:
                self.is_paused = True
                if self.cancel_client.service_is_ready():
                    self.cancel_client.call_async(CancelGoal.Request())
                self.get_logger().warn("[ACIL DURDURMA] Manuel kontrol (x) ile otonom kesif durduruldu!")
        else:  # False = Devam et (r)
            if self.is_paused:
                self.is_paused = False
                self.get_logger().info("[DEVAM] Otonom kesif devam ediyor, yeni sinir araniyor...")
                self.state = 'SEARCH_FRONTIER'

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        if self.state == 'EXPLORING':
            if self.prev_x is not None:
                step = math.hypot(pos.x - self.prev_x, pos.y - self.prev_y)
                if step < 0.5:
                    self.total_distance += step
            self.prev_x = pos.x
            self.prev_y = pos.y

    def scan_callback(self, msg: LaserScan):
        if self.state == 'EXPLORING':
            valid = [r for r in msg.ranges if max(0.20, msg.range_min) < r < msg.range_max]
            if valid:
                m = min(valid)
                if m < self.min_obstacle_dist:
                    self.min_obstacle_dist = m

    def find_frontiers(self):
        """
        Haritada bos (0) ile bilinmeyen (-1) bolgelerin sinirlarini (frontiers) tespit eder.
        Kumeleyip engellerden guvenli mesafedeki en yakin/en buyuk sinir hucresini dondurur.
        """
        if self.map_msg is None:
            return None

        info = self.map_msg.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = self.map_msg.data

        # 1. Sinir hucrelerini bul (BOS alan ve komsu en az 1 BILINMEYEN hucre, engellerden guvenli uzaklikta)
        frontier_cells = set()
        safety_cell_radius = 4  # ~20 cm engel guvenlik mesafesi

        for y in range(safety_cell_radius, h - safety_cell_radius):
            row_idx = y * w
            for x in range(safety_cell_radius, w - safety_cell_radius):
                idx = row_idx + x
                if data[idx] == 0:  # Bos alan
                    # 8 komsuda bilinmeyen (-1) var mi?
                    has_unknown = False
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if dx == 0 and dy == 0:
                                continue
                            if data[(y + dy) * w + (x + dx)] == -1:
                                has_unknown = True
                                break
                        if has_unknown:
                            break

                    if not has_unknown:
                        continue

                    # Engel guvenlik kontrolu (safety_cell_radius icinde engel olmamali)
                    near_obstacle = False
                    for dy in range(-safety_cell_radius, safety_cell_radius + 1):
                        for dx in range(-safety_cell_radius, safety_cell_radius + 1):
                            val = data[(y + dy) * w + (x + dx)]
                            if val > 50:
                                near_obstacle = True
                                break
                        if near_obstacle:
                            break

                    if not near_obstacle:
                        frontier_cells.add((x, y))

        if not frontier_cells:
            return None

        # 2. Sinir hucrelerini kumele (Connected Components BFS)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = deque([cell])
            visited.add(cell)

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))

                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nbr = (cx + dx, cy + dy)
                        if nbr in frontier_cells and nbr not in visited:
                            visited.add(nbr)
                            queue.append(nbr)

            # Gurultuyu filtrele (en az 4 hucrelik kume)
            if len(cluster) >= 4:
                clusters.append(cluster)

        if not clusters:
            return None

        # 3. Kumelerden guvenli hucre sec ve adaylari olustur
        candidates = []
        now = time.time()
        for cl in clusters:
            avg_x = sum(c[0] for c in cl) / len(cl)
            avg_y = sum(c[1] for c in cl) / len(cl)

            # Merkez noktaya en yakin GERCEK bos hucreyi sec
            best_cell = min(cl, key=lambda c: (c[0] - avg_x)**2 + (c[1] - avg_y)**2)
            wx = ox + (best_cell[0] + 0.5) * res
            wy = oy + (best_cell[1] + 0.5) * res

            # Kara listedeki noktalara yakin mi?
            if any(math.hypot(wx - bx, wy - by) < 0.6 for bx, by in self.blacklist):
                continue

            # Son 45 saniyede ziyaret edilmis noktalara yakin mi?
            if any(math.hypot(wx - vx, wy - vy) < 0.7 for vx, vy, t in self.visited_goals if (now - t) < 45.0):
                continue

            dist = math.hypot(wx - self.current_x, wy - self.current_y)
            # Buyuk kumelere ve makul mesafedeki noktalara oncelik ver
            # (Buyuk kume = daha cok kesfedilmemis alan)
            score = -0.15 * len(cl) + 0.8 * dist
            candidates.append((score, dist, wx, wy, len(cl)))

        if not candidates:
            return None

        # Robotun hemen dibindeki (0.75m'den yakin) sinirlara gitmek yerine
        # oncelikle en az 0.75 metre uzaktaki sinirlara oncelik ver.
        # Boylece robot yerinde saymaz, ileriye dogru surus yapar.
        far_candidates = [c for c in candidates if c[1] >= 0.75]
        pool = far_candidates if far_candidates else candidates

        pool.sort(key=lambda item: item[0])
        best = pool[0]
        return best[2], best[3]  # wx, wy

    def exploration_loop(self):
        now = time.time()

        if self.is_paused:
            return

        if self.state == 'WAIT_MAP':
            if self.map_msg is not None and self.current_x is not None and self.goal_pub.get_subscription_count() > 0:
                self.get_logger().info("SLAM Haritasi, Odometri ve Nav2 baglantisi hazir! Kesif baslatiliyor...")
                self.start_time = now
                self.prev_x = self.current_x
                self.prev_y = self.current_y
                self.state = 'SEARCH_FRONTIER'

        elif self.state == 'SEARCH_FRONTIER':
            frontier = self.find_frontiers()
            if frontier is None:
                self.get_logger().info("Tum ortam basariyla kesfedildi! Hicbir bilinmeyen sinir kalmadi.")
                self.finish_exploration()
                return

            self.target_x, self.target_y = frontier
            self.send_goal(self.target_x, self.target_y)
            self.target_sent_time = now
            self.last_progress_x = self.current_x
            self.last_progress_y = self.current_y
            self.last_progress_time = now
            self.state = 'EXPLORING'

        elif self.state == 'EXPLORING':
            dist_to_target = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)

            # Robot henuz harekete baslamadiysa 2.5 saniyede bir hedefi tekrarla
            if self.total_distance < 0.05 and (now - self.target_sent_time >= 2.5):
                self.send_goal(self.target_x, self.target_y)
                self.target_sent_time = now

            # Robot hareketini takip et
            progress = math.hypot(self.current_x - self.last_progress_x, self.current_y - self.last_progress_y)
            if progress >= 0.15:
                self.last_progress_x = self.current_x
                self.last_progress_y = self.current_y
                self.last_progress_time = now

            if now - self.last_log_time >= 2.5:
                self.get_logger().info(
                    f"Kesif Hedefine: {dist_to_target:.2f} m | Toplam Yol: {self.total_distance:.2f} m"
                )
                self.last_log_time = now

            # Hedefe yeterince yaklasti mi? (Gercek Nav2 toleransi 0.35m)
            if dist_to_target <= 0.35:
                self.get_logger().info(f"Bolge kesfedildi ({self.target_x:.2f}, {self.target_y:.2f}), yeni sinir araniyor...")
                self.visited_goals.append((self.target_x, self.target_y, now))
                self.state = 'SEARCH_FRONTIER'

            else:
                max_goal_duration = max(30.0, dist_to_target / 0.15 + 15.0)
                stuck = (now - self.last_progress_time > 10.0)
                timeout = (now - self.target_sent_time > max_goal_duration)

                if stuck or timeout:
                    reason = "ilerleme saglanamadi (takilma)" if stuck else "sure asimi"
                    self.get_logger().warn(f"Noktaya ulasilamadi ({reason}), kara listeye alindi. Baska sinira geciliyor...")
                    self.blacklist.append((self.target_x, self.target_y))
                    self.state = 'SEARCH_FRONTIER'

    def send_goal(self, gx, gy):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = gx
        goal_msg.pose.position.y = gy
        goal_msg.pose.position.z = 0.0

        cur_x = self.current_x if self.current_x is not None else 0.0
        cur_y = self.current_y if self.current_y is not None else 0.0
        target_yaw = math.atan2(gy - cur_y, gx - cur_x)
        goal_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"--> Yeni Kesif Hedefi: ({gx:.2f}, {gy:.2f})")

    def save_map_to_disk(self):
        """Haritayi standart .pgm ve .yaml formatinda diske kaydeder."""
        if self.map_msg is None:
            return

        maps_dir = os.path.expanduser('~/Masaüstü/ros2_deneme/src/otonom_surus/maps')
        os.makedirs(maps_dir, exist_ok=True)
        base_name = f"senaryo{self.senaryo_no}_{self.scenario['ad']}_harita"

        pgm_path = os.path.join(maps_dir, f"{base_name}.pgm")
        yaml_path = os.path.join(maps_dir, f"{base_name}.yaml")

        info = self.map_msg.info
        w, h = info.width, info.height
        data = self.map_msg.data

        # PGM (P5 Binary Netpbm)
        with open(pgm_path, 'wb') as f:
            header = f"P5\n{w} {h}\n255\n".encode('ascii')
            f.write(header)
            # ROS OccupancyGrid alt satirdan baslar
            pixels = bytearray(w * h)
            for y in range(h):
                for x in range(w):
                    val = data[(h - 1 - y) * w + x]
                    if val == 0:
                        pix = 254  # Beyaz: Bos alan
                    elif val == 100:
                        pix = 0    # Siyah: Engel
                    else:
                        pix = 205  # Gri: Bilinmeyen
                    pixels[y * w + x] = pix
            f.write(pixels)

        # YAML
        yaml_content = {
            'image': f"{base_name}.pgm",
            'mode': 'trinary',
            'resolution': float(info.resolution),
            'origin': [float(info.origin.position.x), float(info.origin.position.y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_content, f)

        self.get_logger().info(f"Harita basariyla kaydedildi:\n  PGM : {pgm_path}\n  YAML: {yaml_path}")

    def finish_exploration(self):
        self.state = 'FINISHED'
        duration = time.time() - self.start_time

        self.save_map_to_disk()

        report = f"""
============================================================
           OTONOM KESIF VE HARITALAMA RAPORU
============================================================
Senaryo            : {self.senaryo_no} ({self.scenario['ad']})
Durum              : %100 TAMAMLANDI (Tum Sinirlar Kesfedildi)
Toplam Kesif Suresi: {duration:.2f} saniye ({duration/60.0:.1f} dakika)
Kat Edilen Mesafe  : {self.total_distance:.2f} metre
Harita Boyutu      : {self.map_msg.info.width} x {self.map_msg.info.height} hucre
Cozunurluk         : {self.map_msg.info.resolution} m/hucre
============================================================
"""
        self.get_logger().info(report)

        log_file = os.path.expanduser('~/Masaüstü/ros2_deneme/deney_sonuclari.txt')
        with open(log_file, 'a') as f:
            f.write(report)


def main(args=None):
    rclpy.init(args=args)
    node = OtonomKesif()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
