#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ----------------- Parameters -----------------
        self.declare_parameter('r_left', 0.048082)
        self.declare_parameter('r_right', 0.047918)
        self.declare_parameter('b', 0.287)
        self.declare_parameter('method', 'exact')  # 'euler', 'rk2', or 'exact'

        self.r_L = float(self.get_parameter('r_left').value)
        self.r_R = float(self.get_parameter('r_right').value)
        self.b = float(self.get_parameter('b').value)
        self.method = str(self.get_parameter('method').value).lower()

        # ----------------- State -----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_phi_L = None
        self.prev_phi_R = None
        self.initialized = False

        # Joint names to look up
        self.left_joint_name = 'left_wheel_joint'
        self.right_joint_name = 'right_wheel_joint'

        # ----------------- Publishers & TF -----------------
        self.odom_pub = self.create_publisher(Odometry, '/odom_own', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ----------------- Subscription -----------------
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.get_logger().info(
            f"OdometryNode running with method={self.method}, "
            f"r_L={self.r_L:.6f}, r_R={self.r_R:.6f}, b={self.b:.4f}"
        )

    def joint_state_cb(self, msg: JointState):
        # 1. Find joint indices by name
        try:
            left_idx = msg.name.index(self.left_joint_name)
            right_idx = msg.name.index(self.right_joint_name)
        except ValueError:
            return

        phi_L = msg.position[left_idx]
        phi_R = msg.position[right_idx]

        # 2. Initialize on first message
        if not self.initialized:
            self.prev_phi_L = phi_L
            self.prev_phi_R = phi_R
            self.initialized = True
            return

        # 3. Compute joint displacement and wheel arc lengths
        d_phi_L = phi_L - self.prev_phi_L
        d_phi_R = phi_R - self.prev_phi_R
        self.prev_phi_L = phi_L
        self.prev_phi_R = phi_R

        ds_L = self.r_L * d_phi_L
        ds_R = self.r_R * d_phi_R

        # Arc length ds and heading change dtheta
        ds = 0.5 * (ds_R + ds_L)
        dtheta = (ds_R - ds_L) / self.b

        # 4. Integrate pose using chosen scheme (from class slides 28-38)
        theta_k = self.theta

        if self.method == 'euler':
            # Euler / Rectangular: Heading frozen at start
            self.x += ds * math.cos(theta_k)
            self.y += ds * math.sin(theta_k)
            self.theta += dtheta

        elif self.method == 'rk2' or self.method == 'midpoint':
            # Midpoint / RK2: Heading at halfway direction
            mid_theta = theta_k + 0.5 * dtheta
            self.x += ds * math.cos(mid_theta)
            self.y += ds * math.sin(mid_theta)
            self.theta += dtheta

        elif self.method == 'exact':
            # Exact circular arc integration
            if abs(dtheta) < 1e-6:
                # Limit as dtheta -> 0 converges to RK2 (sinc -> 1)
                mid_theta = theta_k + 0.5 * dtheta
                self.x += ds * math.cos(mid_theta)
                self.y += ds * math.sin(mid_theta)
            else:
                sinc = math.sin(0.5 * dtheta) / (0.5 * dtheta)
                mid_theta = theta_k + 0.5 * dtheta
                self.x += ds * sinc * math.cos(mid_theta)
                self.y += ds * sinc * math.sin(mid_theta)
            self.theta += dtheta

        elif self.method == 'trapezoidal':
            # Trapezoidal: Average of start and end heading directions
            self.x += 0.5 * ds * (math.cos(theta_k) + math.cos(theta_k + dtheta))
            self.y += 0.5 * ds * (math.sin(theta_k) + math.sin(theta_k + dtheta))
            self.theta += dtheta

        # Normalize heading to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 5. Publish Odometry and TF
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)

        # /odom_own message
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom_msg)

        # odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()