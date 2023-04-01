#! /usr/bin/env python3

import utils.dwa
import rclpy
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist, PoseStamped, Point, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.srv import GetMap
from cartographer_ros_msgs.srv import StartTrajectory
class ITAVController:
  def __init__(self, node):
    print("controller initialized")
    self.current_pose = None
    self.goal_pose = None
    self.grid = None
    self.odom_sub = node.create_subscription(Odometry, 'itav_agv/odometry/filtered', self.odom_callback, 10)
    self.tracked_pose = node.create_subscription(PoseWithCovarianceStamped, 'itav_agv/global_pose', self.pose_callback, 10)
    self.goal_sub = node.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
    self.cmd_vel_pub = node.create_publisher(Twist, 'itav_agv/cmd_vel', 10)
    self.trajectory_pub = node.create_publisher(Path, 'trajectory1', 10)
    # self.grid_map_sub = node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.grid_map_callback, 10)
    self.grid_map_sub = node.create_subscription(OccupancyGrid, 'map', self.grid_map_callback, 1)
    self.obstacle_sub = node.create_subscription(Float32MultiArray, 'obstacles_state', self.obstacle_callback, 10)
    self.ob_pub = node.create_publisher(MarkerArray, 'obstacles', 10)
    # get map service call
    self.get_map_client = node.create_client(GetMap, '/map_server/map')
    # cartographer start trajectory service call
    self.start_trajectory_client = node.create_client(StartTrajectory, '/start_trajectory')
    # list of trajectories publisher
    self.trajectories_pub = node.create_publisher(MarkerArray, 'trajectory_list1', 10)
    
    self.dwa = utils.dwa.DWA(self.current_pose)
    self.trajectory = np.array([[0,0,0]])

    # obstacles
    self.dwa.config.ob = np.array([[20, 20, 0, 0.2,]
                            ]) 



    while not self.get_map_client.wait_for_service(timeout_sec=1.0):
      print('map ervice not available, waiting again...')
    self.req = GetMap.Request()
    self.future = self.get_map_client.call_async(self.req)
    rclpy.spin_until_future_complete(node, self.future)
    if self.future.result() is not None:
      self.grid = np.array(self.future.result().map.data).reshape(self.future.result().map.info.width, self.future.result().map.info.height).T
      self.dwa.set_map(self.grid)
      print("map received")
    
    # while not self.start_trajectory_client.wait_for_service(timeout_sec=1.0) or self.current_pose is None:
    #   print('cartographer service not available, waiting again...', self.current_pose)
    #   self.current_pose = [20.0, 55.0, 0.0]
    # self.req = StartTrajectory.Request()
    # self.req.configuration_directory = "/home/phoenix/ros2_ws/src/tavil/itav_agv_bringup/config"
    # self.req.configuration_basename = "cartographer_2d.lua"
    # self.req.use_initial_pose = bool(0)
    # self.req.relative_to_trajectory_id = 0
    # self.req.initial_pose.position.x = self.current_pose[0]
    # self.req.initial_pose.position.y = self.current_pose[1]
    # self.req.initial_pose.position.z = 0.0
    # self.req.initial_pose.orientation.x = 0.0
    # self.req.initial_pose.orientation.y = 0.0
    # self.req.initial_pose.orientation.z = 0.0
    # self.req.initial_pose.orientation.w = 1.0
    # self.future = self.start_trajectory_client.call_async(self.req)
    # rclpy.spin_until_future_complete(node, self.future)
    # if self.future.result() is not None:
    #   print("trajectory started")

    # ros2 timer controller
    self.timer = node.create_timer(0.1, self.controller)
    # self.timer = node.create_timer(0.5, self.plotter)

  def obstacle_callback(self, msg):
    # pass
    # first remove the obstacle that is the same as the current robot pose by checking the distance
    print("obstacles size", len(msg.data))
    if len(msg.data) == 0:
       obstacles = np.array([[800, 800, 0, 0.2,], [800, 800, 0, 0.2,], [800, 800, 0, 0.2,]])
    else:
      obstacles = np.array(msg.data).reshape(-1, 4)
      # remove obstacles that are far from current pose
      if self.current_pose is not None:
        obstacles = obstacles[np.linalg.norm(obstacles[:, :2] - self.current_pose[:2], axis=1) < 70]
        obstacles = np.append(obstacles, np.array([[800, 800, 0, 0.2,], [800, 800, 0, 0.2,], [800, 800, 0, 0.2,]]), axis=0)
      # if self.grid is not None:
      #   obstacles = obstacles[self.grid[20*obstacles[:, 0].astype(int), 20*obstacles[:, 1].astype(int)] == 0]
    if self.current_pose is not None:
      d = np.hypot(self.current_pose[0], self.current_pose[1])
      r = 1. 
      x = np.zeros(2)
      x[0] = self.current_pose[0] + r * np.cos(self.current_pose[2])
      x[1] = self.current_pose[1] + r * np.sin(self.current_pose[2])
      obstacles = obstacles[np.linalg.norm(obstacles[:, :2] - x, axis=1) > 0.75]
    self.dwa.config.ob = obstacles
    self.publish_ob()

  def grid_map_callback(self, msg):
    # print("grid map shape", msg.info.width, msg.info.height)
    self.grid = np.array(msg.data).reshape(msg.info.width, msg.info.height)
    origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
    # origin adjustment
    self.grid = np.roll(self.grid, int(origin[0]), axis=0)
    self.grid = np.roll(self.grid, int(origin[1]), axis=1)
    print("grid map shape", self.grid.shape)
    self.dwa.set_map(self.grid)
  # rviz goal_pose subscriber
  def goal_callback(self, msg):

    x = msg.pose.position.x
    y = msg.pose.position.y
    _,_,yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    print("goal_pose received", x, y, yaw)
    self.goal_pose = [x, y, yaw]
    self.dwa.set_goal_pose(self.goal_pose)
    # publish ob as marker array
    
  def publish_ob(self):
    # publish obstacles as marker array
    marker_array = MarkerArray()
    vel_marker_array = MarkerArray()
    for i in range(len(self.dwa.config.ob)):
        marker = Marker()
        vel_marker = Marker()
        marker.header.frame_id = "odom"
        # marker.header.stamp = rclpy.get_clock().now().to_msg()
        # u = [np.random.uniform(-0.75, 0.75), np.random.uniform(-0.5, 0.5)]
        # x = self.dwa.motion(self.dwa.config.ob[i,:], u, 1)
        # self.dwa.config.ob[i,:] = x
        marker.ns = "obstacles"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.dwa.config.ob[i,0])
        marker.pose.position.y = float(self.dwa.config.ob[i,1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        vel_marker.header.frame_id = "odom"
        vel_marker.type = Marker.ARROW
        vel_marker.action = Marker.ADD
        vel_marker.ns = "velocities"
        vel_marker.id = i
        vel_marker.pose.position.x = float(self.dwa.config.ob[i,0])
        vel_marker.pose.position.y = float(self.dwa.config.ob[i,1])
        vel_marker.pose.position.z = 0.5
        yaw = np.arctan2(self.dwa.config.ob[i,3], self.dwa.config.ob[i,2])
        # convert yaw to quaternion
        q = quaternion_from_euler(0, 0, yaw)
        vel_marker.pose.orientation.x = q[0]
        vel_marker.pose.orientation.y = q[1]
        vel_marker.pose.orientation.z = q[2]
        vel_marker.pose.orientation.w = q[3]
        vel_marker.scale.x = float(np.hypot(self.dwa.config.ob[i,2], self.dwa.config.ob[i,3])) +0.00001
        vel_marker.scale.y = 0.1
        vel_marker.scale.z = 0.1
        vel_marker.color.a = 1.0
        vel_marker.color.r = 0.0
        vel_marker.color.g = 1.0
        vel_marker.color.b = 0.0
        marker_array.markers.append(marker)
        vel_marker_array.markers.append(vel_marker)
    self.ob_pub.publish(marker_array)
    self.ob_pub.publish(vel_marker_array)

  # ros2 subscriber for odom
  def odom_callback(self, msg):

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    self.current_pose = [x, y, yaw]
    self.dwa.set_current_pose(self.current_pose)
    vx = msg.twist.twist.linear.x
    az = msg.twist.twist.angular.z
    self.dwa.set_cmd_vel([vx, yaw])
    # print("current_pose received", x, y, yaw)

  def pose_callback(self, msg):
    # print("pose callback")
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    self.current_pose = [x, y, yaw]
    self.dwa.set_current_pose(self.current_pose)
    # print("current_pose received", x, y, yaw)
    
  # controller timer
  def controller(self):
    # print("controller called")
    if self.goal_pose != None and self.current_pose != None:

      # print("current_pose: ", self.current_pose)
      # TODO: goal is far away break in small goal lists based on dubins path
      
      self.dwa.set_goal_pose(self.goal_pose)
      u, predicted_trajectory = self.dwa.dwa_control()
      # self.dwa.set_cmd_vel(u)
      trajectory_vis = Path()

      trajectory_vis.header.frame_id = "map"
      # trajectory_vis.header.stamp = rclpy.get_clock().now().to_msg()
      for i in range(len(predicted_trajectory)):
          pose = PoseStamped()
          pose.header.frame_id = "map"
          pose.pose.position.x = predicted_trajectory[i][0]
          pose.pose.position.y = predicted_trajectory[i][1]
          trajectory_vis.poses.append(pose)
      self.trajectory_pub.publish(trajectory_vis)
      trajectories_msg = MarkerArray() 
      trajectory_msg = Marker()

      trajectories = self.dwa.get_trajectories()

      for i in range(len(trajectories)):
          trajectory_msg = Marker()
          trajectory_msg.header.frame_id = "map"
          trajectory_msg.ns = "trajectories"
          trajectory_msg.id = i
          trajectory_msg.type = Marker.LINE_STRIP
          trajectory_msg.action = Marker.ADD
          trajectory_msg.scale.x = 0.05
          trajectory_msg.color.a = 1.0
          trajectory_msg.color.r = 0.0
          trajectory_msg.color.g = 0.0
          trajectory_msg.color.b = 1.0
          for j in range(len(trajectories[i])):
              pose = Point()
              pose.x = trajectories[i][j][0]
              pose.y = trajectories[i][j][1]
              pose.z = 0.0
              trajectory_msg.points.append(pose)
          trajectories_msg.markers.append(trajectory_msg)
      self.trajectories_pub.publish(trajectories_msg)
      
      
      vel = Twist()
      vel.linear.x = u[0]
      vel.angular.z = u[1]
      print("vel: ", vel.linear.x, vel.angular.z)
      print("goal_pose: ", self.goal_pose)
      self.publish_vel(u)

      dist_to_goal = np.hypot(self.goal_pose[0] - self.current_pose[0], self.goal_pose[1] -self.current_pose[1])
      if dist_to_goal < 0.5:
          print("goal reached")
          self.goal_pose = None
          self.publish_vel([0.0, 0.0])
    else:
      print("waiting for goal_pose", self.goal_pose, self.current_pose)
  
  def plotter(self):
    if self.goal_pose != None and self.current_pose != None:
       self.dwa.plot_all()
       
  # publish velocity
  def publish_vel(self, vel):
      msg = Twist()
      msg.linear.x = vel[0]
      msg.angular.z = vel[1]
      self.cmd_vel_pub.publish(msg)

def main(args=None):
      
      rclpy.init(args=args)
      node = rclpy.create_node('itav_agv_controller')
      controller = ITAVController(node)
      rclpy.spin(node)

if __name__ == '__main__':
    main()