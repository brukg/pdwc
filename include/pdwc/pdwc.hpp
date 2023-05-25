#ifndef ITAV_AGV_CONTROLLER_H
#define ITAV_AGV_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>
#include "pdwc/dwa.hpp"
#include "pdwc/a_star.hpp"
#include <Eigen/Dense>

class ITAVController : public rclcpp::Node
{
public:
    ITAVController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void obstacleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void controller();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tracked_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr respones_time_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ob_pub;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_client;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_client;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectories_pub;
    rclcpp::TimerBase::SharedPtr timer;

    std::shared_ptr<DWA> dwa;
    Eigen::ArrayXXd trajectory;
    Eigen::MatrixXd grid;
    Eigen::VectorXd current_pose;
    Eigen::Vector3d goal_pose;
    Eigen::MatrixXd obstacles;
    std::string footprint_string;
    Eigen::Vector2d map_origin;
    int rate;
    double map_resolution;
    int map_height, map_width;
    std::vector<std::array<float, 2>> footprint;
    std::shared_ptr<AStar> astar;
};

#endif // ITAV_AGV_CONTROLLER_H
