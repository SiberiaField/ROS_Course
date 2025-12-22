#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs_py import point_cloud2


class DepthDodger(Node):
    def __init__(self):
        super().__init__('depth_dodger')
        self.pub = self.create_publisher(Twist, '/crab_bot/cmd_vel', 10)
        self.sub = self.create_subscription(PointCloud2, 'crab_bot/depth_camera/points', self.pc_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.obstacle_detected = False
        self.last_pc_time = self.get_clock().now()

    def pc_callback(self, msg: PointCloud2):
        self.last_pc_time = self.get_clock().now()
        try:
            pc = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), reshape_organized_cloud=True)
        except Exception as e:
            self.get_logger().warn(f'Failed to read point cloud: {e}')
            return

        valid = ~np.isnan(pc[..., 0])
        if not np.any(valid):
            self.obstacle_detected = False
            return

        height, width = pc.shape[:2]
        center_y = width // 2
        y_range = slice(max(0, center_y - 150), min(width, center_y + 150))

        h_slice = slice(0, int(height * 2/3))
        front_slice = pc[h_slice, y_range, :]
        valid_front = valid[h_slice, y_range]
        front_points = front_slice[valid_front]

        if len(front_points) == 0:
            self.get_logger().debug('No valid front points')
            self.obstacle_detected = False
            return

        distances = front_points[:, 0]
        z_values = front_points[:, 2]

        close_mask = (distances > 0.2) & (distances < 0.8) & (z_values > -0.1)
        close_points = distances[close_mask]

        if len(close_points) > 10:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        time_since_pc = (self.get_clock().now() - self.last_pc_time).nanoseconds / 1e9
        if time_since_pc > 1.0:
            self.obstacle_detected = True

        twist = Twist()
        twist.linear.x = 0.0 if self.obstacle_detected else 0.25
        twist.angular.z = 0.0
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DepthDodger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
