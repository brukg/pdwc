#! /usr/bin/env python3
# Ros2 subscribe to odom and publish as gemetry_msgs/PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import math

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
        # transform  the pose
        pose = np.array([[T.transform.translation.x + msg.pose.pose.position.x,
                         T.transform.translation.y + msg.pose.pose.position.y,
                         T.transform.translation.z + msg.pose.pose.position.z]])
        # transform the orientation
        # q = msg.pose.pose.orientation
        # q = np.array([q.w, q.y, q.z, q.x])
        # R = np.array([[1-2*(q[2]**2+q[3]**2), 2*(q[1]*q[2]-q[3]*q[0]), 2*(q[1]*q[3]+q[2]*q[0])],
        #                 [2*(q[1]*q[2]+q[3]*q[0]), 1-2*(q[1]**2+q[3]**2), 2*(q[2]*q[3]-q[1]*q[0])],
        #                 [2*(q[1]*q[3]-q[2]*q[0]), 2*(q[2]*q[3]+q[1]*q[0]), 1-2*(q[1]**2+q[2]**2)]])
        # print(R)
        # rot = np.array([T.transform.rotation.w, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.x])
        # R_ = np.array([[1-2*(rot[2]**2+rot[3]**2), 2*(rot[1]*rot[2]-rot[3]*rot[0]), 2*(rot[1]*rot[3]+rot[2]*rot[0])],
        #                 [2*(rot[1]*rot[2]+rot[3]*rot[0]), 1-2*(rot[1]**2+rot[3]**2), 2*(rot[2]*rot[3]-rot[1]*rot[0])],
        #                 [2*(rot[1]*rot[3]-rot[2]*rot[0]), 2*(rot[2]*rot[3]+rot[1]*rot[0]), 1-2*(rot[1]**2+rot[2]**2)]])
        # print(R_)
        
        # # R = R_ @ R
        # print(R)
        # q = np.array([math.sqrt(1+R[0,0]+R[1,1]+R[2,2])/2,
        #                 (R[2,1]-R[1,2])/(4*q[0]),
        #                 (R[0,2]-R[2,0])/(4*q[0]),
        #                 (R[1,0]-R[0,1])/(4*q[0])])
        # self.pose.pose.pose.orientation.x = q[1]
        # self.pose.pose.pose.orientation.y = q[2]
        # self.pose.pose.pose.orientation.z = q[3]
        # self.pose.pose.pose.orientation.w = q[0]
        # self.pose.pose.pose.position.x = pose[0,0]
        # self.pose.pose.pose.position.y = pose[0,1]
        # self.pose.pose.pose.position.z = pose[0,2]


        self.pose.pose.pose.position.x = msg.pose.pose.position.x + T.transform.translation.x
        self.pose.pose.pose.position.y = msg.pose.pose.position.y + T.transform.translation.y
        self.pose.pose.pose.position.z = msg.pose.pose.position.z + T.transform.translation.z
        self.pose.pose.pose.orientation.x = msg.pose.pose.orientation.x #+ T.transform.rotation.x
        self.pose.pose.pose.orientation.y = msg.pose.pose.orientation.y #+ T.transform.rotation.y
        self.pose.pose.pose.orientation.z = msg.pose.pose.orientation.z #+ T.transform.rotation.z
        self.pose.pose.pose.orientation.w = msg.pose.pose.orientation.w #+ T.transform.rotation.w
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