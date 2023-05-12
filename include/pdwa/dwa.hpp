#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include "omp.h"
using Traj = std::vector<std::array<float, 5>>;
using Obstacle = Eigen::MatrixXd;
using State = std::array<float, 5>;
using Window = std::array<float, 4>;
using Point = Eigen::Vector3d;
using Control = std::array<float, 2>;
using Result = std::pair<Control, Traj>;
class Config 
{
  public:
    // Config();
    // ~Config(){};
    Obstacle ob;

    float max_speed = 1.52;
    float min_speed = -1.52;
    float max_yawrate = 90.0 * M_PI / 180.0;
    float max_accel = 16.0;
    float robot_radius = 1.5;
    float footprint[4][2] = {{1.7, 0.6}, {1.7, -0.6}, {-0.6, -0.6}, {-0.6, 0.6}};
    float obstacle_margin = 0.2;
    float max_dyawrate = 750.0 * M_PI / 180.0;

    float v_reso = 0.3;
    float yawrate_reso = 5 * M_PI / 180.0;

    float dt = 0.05;
    float predict_time = 3.0;
    float to_goal_cost_gain = 1.20;
    float speed_cost_gain = 7.0;
    float obstacle_cost_gain = 1.1;
    float collision_threshold = 0.001;
    float obstacle_radius = 0.5;
    float reverse_penality = 2.0;
    float goal_tolerance = 0.5;
};



class DWA 
{
  public:
    DWA(Eigen::VectorXd current_pose);
    ~DWA(){};
    State motionModel(State x, Control u, float dt);
    Window dynamicWindow(State x, Config config);
    Traj calc_trajectory(State x, float v, float y, Config config);
    float costObstacle(Traj traj, Config config);
    float costGoal(Traj traj, Point goal, Config config);
    Traj finalInput(State x, Control& u, Window dw, Config config, Point goal);
    Result DWAControl();

    void setState(Eigen::VectorXd current_pose);
    void setMap(Eigen::MatrixXd map, double resolution, Eigen::Vector2d origin);
    void setGoal(Point goal);
    void setObstacles(Eigen::MatrixXd ob);
    void setFootprint(float footprint[][2]);
    void getTrajectories(std::vector<Traj> &trajectories);
    void getFootprint(std::vector<std::array<float, 2>> &footprint);
    Config config;

  private:
    Eigen::MatrixXd map_;
    double resolution;
    // Eigen::Vector3d current_pose;
    Point goal;
    State x;
    Control u;
    Window dw;
    Traj traj;
    Obstacle ob;
    std::vector<Traj> trajectories;
    Eigen::Vector2d origin;
};