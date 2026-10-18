#!/usr/bin/env python3
import math
import csv
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OpenLoopSquare(Node):
    def __init__(self):
        super().__init__('open_loop_square')

        # ----------------- Parameters -----------------
        # Side length L (m), speed v (m/s), turn rate w (rad/s), direction (ccw or cw)
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('turn_rate', 0.5)
        self.declare_parameter('direction', 'ccw')
        self.declare_parameter('csv_filename', 'open_loop_trajectory.csv')

        self.L = float(self.get_parameter('side_length').value)
        self.v = float(self.get_parameter('speed').value)
        self.w = float(self.get_parameter('turn_rate').value)
        self.direction = str(self.get_parameter('direction').value).lower()
        self.csv_path = str(self.get_parameter('csv_filename').value)

        # Angular velocity sign: CCW is +w, CW is -w
        self.turn_sign = 1.0 if self.direction == 'ccw' else -1.0

        # Motion durations (seconds)
        self.t_straight = self.L / self.v
        self.t_turn = (math.pi / 2.0) / self.w

        # State machine tracking: 4 sides, alternating STRAIGHT and TURN
        # Sequence: [STRAIGHT, TURN] repeated 4 times, then STOP
        self.current_state = 'STRAIGHT'
        self.side_count = 0
        self.state_start_time = None

        # Latest pose storage for CSV logging
        self.latest_odom = None
        self.latest_gt = None

        # ----------------- CSV File Setup -----------------
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'sim_time_sec',
            'odom_x', 'odom_y', 'odom_yaw',
            'gt_x', 'gt_y', 'gt_yaw'
        ])

        # ----------------- Subscriptions & Publishers -----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_callback, 10)

        # Main control and logging loop at 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f"OpenLoopSquare initialized: L={self.L}m, v={self.v}m/s, w={self.w}rad/s, dir={self.direction}")

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def gt_callback(self, msg: Odometry):
        self.latest_gt = msg

    def log_csv(self, now):
        sim_sec = now.nanoseconds / 1e9

        ox, oy, oyaw = 0.0, 0.0, 0.0
        if self.latest_odom is not None:
            ox = self.latest_odom.pose.pose.position.x
            oy = self.latest_odom.pose.pose.position.y
            oyaw = self.quat_to_yaw(self.latest_odom.pose.pose.orientation)

        gx, gy, gyaw = 0.0, 0.0, 0.0
        if self.latest_gt is not None:
            gx = self.latest_gt.pose.pose.position.x
            gy = self.latest_gt.pose.pose.position.y
            gyaw = self.quat_to_yaw(self.latest_gt.pose.pose.orientation)

        self.csv_writer.writerow([sim_sec, ox, oy, oyaw, gx, gy, gyaw])

    def control_loop(self):
        now = self.get_clock().now()

        # Handle startup delay until valid clock/time arrives
        if now.nanoseconds == 0:
            return

        if self.state_start_time is None:
            self.state_start_time = now

        elapsed = (now - self.state_start_time).nanoseconds / 1e9
        twist = Twist()

        if self.current_state == 'STRAIGHT':
            if elapsed < self.t_straight:
                twist.linear.x = self.v
                twist.angular.z = 0.0
            else:
                self.current_state = 'TURN'
                self.state_start_time = now
                self.get_logger().info(f"Completed straight side {self.side_count + 1}/4. Starting turn.")

        elif self.current_state == 'TURN':
            if elapsed < self.t_turn:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_sign * self.w
            else:
                self.side_count += 1
                if self.side_count < 4:
                    self.current_state = 'STRAIGHT'
                    self.state_start_time = now
                    self.get_logger().info(f"Completed turn {self.side_count}/4. Starting next straight segment.")
                else:
                    self.current_state = 'DONE'
                    self.state_start_time = now
                    self.get_logger().info("Finished all 4 sides of the square. Stopping.")

        elif self.current_state == 'DONE':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.log_csv(now)
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f"Log written to {self.csv_path}. Shutting down node.")
            self.timer.cancel()
            raise SystemExit

        self.cmd_pub.publish(twist)
        self.log_csv(now)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopSquare()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()