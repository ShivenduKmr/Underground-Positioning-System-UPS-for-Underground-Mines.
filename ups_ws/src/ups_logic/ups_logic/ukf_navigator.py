import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

class UPSNavigator(Node):
    def __init__(self):
        super().__init__('ups_navigator')
        # 1. UKF Setup
        sigmas = MerweScaledSigmaPoints(6, alpha=.1, beta=2., kappa=0)
        self.ukf = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=0.1, fx=self.predict, hx=self.meas, points=sigmas)
        
        # 2. ROS Connections
        self.cmd_pub = self.create_publisher(Twist, '/model/ups_drone/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/ups_drone/odometry', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        
        # 3. Mission Parameters
        self.state = "TAKEOFF"
        self.target_z = 1.0  # 1 meter hover
        self.goal_x = 12.0   # Mapping depth
        self.odom_received = False
        self.lidar_data = None

        self.create_timer(0.1, self.loop)
        self.get_logger().info("--- UPS MISSION: UKF ENGINE READY ---")

    def predict(self, x, dt): return x + np.array([x[3]*dt, x[4]*dt, x[5]*dt, 0, 0, 0])
    def meas(self, x): return x[0:3]

    def odom_cb(self, msg):
        self.odom_received = True
        z = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.ukf.predict()
        self.ukf.update(z)

    def lidar_cb(self, msg): self.lidar_data = msg

    def loop(self):
        if not self.odom_received or self.lidar_data is None: return
        
        x, y, z = self.ukf.x[0], self.ukf.x[1], self.ukf.x[2]
        cmd = Twist()

        if self.state == "TAKEOFF":
            if z < 0.9:
                cmd.linear.z = 0.3
                self.get_logger().info(f"TAKEOFF: Altitude {z:.2f}m")
            else:
                self.get_logger().warn("--- HOVER ESTABLISHED: STARTING MAPPING ---")
                self.state = "EXPLORE"

        elif self.state == "EXPLORE":
            # Potential Field for Obstacle Avoidance
            f_att_x, f_att_y = 0.5, -y * 0.5
            f_rep_x, f_rep_y = 0.0, 0.0
            ranges = np.array(self.lidar_data.ranges)
            for i, r in enumerate(ranges):
                if 0.1 < r < 2.5:
                    angle = self.lidar_data.angle_min + (i * self.lidar_data.angle_increment)
                    rep = 2.0 * (1.0/r - 1.0/2.5) * (1.0/r**2)
                    f_rep_x -= rep * np.cos(angle); f_rep_y -= rep * np.sin(angle)
            
            cmd.linear.x = np.clip(f_att_x + f_rep_x, 0.1, 0.5)
            cmd.linear.y = np.clip(f_att_y + f_rep_y, -0.6, 0.6)
            cmd.linear.z = (self.target_z - z) * 0.5 # Maintain 1m height
            
            if x > self.goal_x:
                self.get_logger().error("--- GOAL REACHED: RETURNING TO BASE ---")
                self.state = "RETURN"

        elif self.state == "RETURN":
            dist_home = np.sqrt(x**2 + y**2)
            if dist_home > 0.4:
                cmd.linear.x = -0.6
                cmd.linear.y = -y * 0.5
                cmd.linear.z = (self.target_z - z) * 0.5
            else:
                self.state = "LAND"

        elif self.state == "LAND":
            if z > 0.15:
                cmd.linear.z = -0.2
                self.get_logger().info("LANDING...")
            else:
                cmd.linear.z = 0.0
                self.get_logger().warn("--- MISSION COMPLETE: SECURED ---")
                rclpy.shutdown()

        self.cmd_pub.publish(cmd)

def main(): rclpy.init(); rclpy.spin(UPSNavigator())
if __name__ == '__main__': main()