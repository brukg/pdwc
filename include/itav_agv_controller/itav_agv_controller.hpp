#ifndef ITAV_CONTROLLER_HPP
#define ITAV_CONTROLLER_HPP

#include "itav_agv_controller/dwa.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>

class ITAVController : public rclcpp::Node {
public:
    ITAVController();

private:
    void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void grid_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_ob();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controller();
    void plotter();
    void publish_vel(const std::array<float, 2>& vel);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ob_pub;

    rclcpp::TimerBase::SharedPtr controller_timer;
    rclcpp::TimerBase::SharedPtr plotter_timer;

    DWAPlanner dwa;
    std::vector<float> current_pose;
    std::vector<float> goal_pose;
    std::vector<std::vector<float>> trajectory;
};

#endif // ITAV_CONTROLLER_HPP
