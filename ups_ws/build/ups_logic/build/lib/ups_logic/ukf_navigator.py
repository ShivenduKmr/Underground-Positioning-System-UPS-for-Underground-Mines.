import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

class UKFNavigator(Node):
    def __init__(self):
        super().__init__('ukf_navigator')
        
        # 1. Initialize UKF (State: x, y, z, vx, vy, vz)
        sigmas = MerweScaledSigmaPoints(6, alpha=.1, beta=2., ki=0)
        self.ukf = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=0.1, fx=self.move_model, hx=self.meas_model, points=sigmas)
        
        # Initial Uncertainty
        self.ukf.P *= 0.2
        self.ukf.R *= 0.5 # Measurement trust
        self.ukf.Q *= 0.01 # Process trust
        
        # 2. ROS Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/model/ups_drone/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # 3. Autonomous Mission Settings
        self.target_z = 1.5  # Takeoff height
        self.target_x = 10.0 # Distance to move in tunnel
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("UKF Navigator Initialized. Starting Autonomous Mission...")

    def move_model(self, x, dt):
        """Logic: x_new = x_old + v*dt (The Prediction Step)"""
        f = np.eye(6)
        f[0, 3] = dt; f[1, 4] = dt; f[2, 5] = dt
        return np.dot(f, x)

    def meas_model(self, x):
        """Logic: What the 'Eyes' see (The Correction Step)"""
        return np.array([x[0], x[1], x[2]])

    def imu_callback(self, msg):
        """Predict movement based on IMU Accel"""
        # In a full VIO, we would integrate accel here. 
        # For now, we trigger the UKF Prediction step.
        self.ukf.predict()

    def control_loop(self):
        """The Brain: Compares UKF State to Target and sends Motors commands"""
        curr_pos = self.ukf.x
        
        cmd = Twist()
        
        # Simple Proportional Control Logic
        # 1. Takeoff Logic
        if curr_pos[2] < self.target_z:
            cmd.linear.z = 0.5
        else:
            cmd.linear.z = 0.0
            
        # 2. Forward Movement Logic (Tracing Miner)
        if curr_pos[0] < self.target_x:
            cmd.linear.x = 0.3
            
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Estimated Pos: x={curr_pos[0]:.2f}, z={curr_pos[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = UKFNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
