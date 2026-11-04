import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from action_msgs.srv import CancelGoal
import sys
import select
import termios
import tty
import time

msg = """
==================================================
   TurtleBot3 Akıllı Klavye Kontrolü (Publisher)
==================================================
   [w] : İleri sür
   [s] : Geri sür
   [a] : Sola dön
   [d] : Sağa dön
   
   [x] veya [Boşluk] : ACİL DURDUR (Manuel ve Otonomu Durdur)
   [r]               : Otonom Sürüşe Devam Et (Resume)
   
   * Güvenlik: Ön engel mesafesi < 0.25 m ise çarpışma
     önleyici otomatik fren devreye girer.
   * Otonom Entegrasyonu: 'x' tuşuna basıldığında Nav2 ve
     otonom düğümler anında iptal edilir.
     
   CTRL-C ile çıkabilirsiniz.
==================================================
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_pub = self.create_publisher(Bool, '/acil_durdur', 10)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Hız limitleri
        self.linear_vel = 0.22
        self.angular_vel = 0.65
        self.front_obstacle_dist = float('inf')
        
        print(msg)

    def scan_callback(self, msg: LaserScan):
        # Robotun tam önündeki 40 derecelik açıdaki engeller
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        front_ranges = []
        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment
            # -20 ile +20 derece arasi (ön taraf)
            if -0.35 <= angle <= 0.35:
                r = msg.ranges[i]
                if max(0.20, msg.range_min) < r < msg.range_max:
                    front_ranges.append(r)

        if front_ranges:
            self.front_obstacle_dist = min(front_ranges)
        else:
            self.front_obstacle_dist = float('inf')

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.08)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        last_key_time = 0.0
        is_moving = False
        
        try:
            while rclpy.ok():
                now = time.time()
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self.get_key(settings)
                
                if not key:
                    # Tus birakildiginda robot hala hareket halindeyse (200 ms sonra) bir kez durdur
                    if is_moving and (now - last_key_time >= 0.20):
                        stop_twist = Twist()
                        self.publisher_.publish(stop_twist)
                        is_moving = False
                        print("\r[DURDU] Tus birakildi, robot durdu.                          ", end="")
                    continue  # Diger zamanlarda hicbir mesaj yayinlama (otonom suruse engel olmamak icin)

                twist = Twist()
                send = False

                if key == 'w':
                    if self.front_obstacle_dist < 0.25:
                        print(f"\r[GÜVENLİK FRENİ] Ön engel çok yakın ({self.front_obstacle_dist:.2f} m)! İleri gidilemez.   ", end="")
                        twist.linear.x = 0.0
                    else:
                        twist.linear.x = self.linear_vel
                        print(f"\r--> İleri (Hız: {self.linear_vel:.2f} m/s | Ön Mesafe: {self.front_obstacle_dist:.2f} m)   ", end="")
                    twist.angular.z = 0.0
                    send = True

                elif key == 's':
                    twist.linear.x = -self.linear_vel
                    twist.angular.z = 0.0
                    print(f"\r--> Geri (Hız: -{self.linear_vel:.2f} m/s)   ", end="")
                    send = True

                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_vel
                    print(f"\r--> Sola Dönüş (Açısal Hız: {self.angular_vel:.2f} rad/s)   ", end="")
                    send = True

                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_vel
                    print(f"\r--> Sağa Dönüş (Açısal Hız: -{self.angular_vel:.2f} rad/s)   ", end="")
                    send = True

                elif key == ' ' or key == 'x':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    send = True

                    # 1. Acil durdurma sinyali yayinla (navigator ve kesif dugumleri icin)
                    stop_msg = Bool()
                    stop_msg.data = True
                    self.emergency_pub.publish(stop_msg)

                    # 2. Nav2 uzerindeki aktif gorevi iptal et
                    if self.cancel_client.service_is_ready():
                        self.cancel_client.call_async(CancelGoal.Request())

                    print("\r[ACIL DURDURMA] Robot ve OTONOM surus TAMAMEN durduruldu! (Devam icin: [r])   ", end="")

                elif key == 'r':
                    # Otonom suruse devam etme izni ver
                    resume_msg = Bool()
                    resume_msg.data = False
                    self.emergency_pub.publish(resume_msg)
                    print("\r[DEVAM] Otonom suruse devam izni verildi.                                    ", end="")

                elif key == '\x03':  # Ctrl+C
                    print("\nÇıkış yapılıyor...")
                    break

                if send:
                    self.publisher_.publish(twist)
                    last_key_time = now
                    if key in ('w', 's', 'a', 'd'):
                        is_moving = True
                    else:
                        is_moving = False
                
        except Exception as e:
            print(f"\nHata: {e}")
        finally:
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
