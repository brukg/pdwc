#! /usr/bin/env python3
# Ros2 subscribe to odom and publish as gemetry_msgs/PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose')
        self.subscription = self.create_subscription(Odometry, 'odom', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'global_pose', 1)
        self.pose = PoseWithCovarianceStamped()
        self.pose.pose.covariance = np.array([0.01, 0, 0, 0, 0, 0,
                                              0, 0.01, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0.01, 0, 0,
                                              0, 0, 0, 0, 0.01, 0,
                                              0, 0, 0, 0, 0, 0.01])
        self.pose.header.frame_id = "map"
        self.pose.pose.pose.position.x = float(0)
        self.pose.pose.pose.position.y = float(0)
        self.pose.pose.pose.position.z = float(0)
        self.pose.pose.pose.orientation.x = float(0)
        self.pose.pose.pose.orientation.y = float(0)
        self.pose.pose.pose.orientation.z = float(0)
        self.pose.pose.pose.orientation.w = float(1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):

        self.tf_buffer.wait_for_transform_async("map", msg.header.frame_id, rclpy.time.Time())
        T = self.tf_buffer.lookup_transform("map", msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

        # Build a PoseStamped from odom and transform it properly into map frame
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose
        map_pose = do_transform_pose_stamped(odom_pose, T)

        self.pose.pose.pose = map_pose.pose
        self.pose.header.stamp = msg.header.stamp
        self.publisher_.publish(self.pose)
    
def main(args=None):
    rclpy.init(args=args)

    odom_to_pose = OdomToPose()

    rclpy.spin(odom_to_pose)

    odom_to_pose.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()