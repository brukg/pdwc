#include "itav_agv_controller/itav_agv_controller.hpp"


ITAVController::ITAVController() : Node("itav_agv_controller")
{
    RCLCPP_INFO(this->get_logger(), "controller initialized");

    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "itav_agv/odometry/filtered",
        10,
        std::bind(&ITAVController::odom_callback, this, std::placeholders::_1));
    tracked_pose = create_subscription<geometry_msgs::msg::PoseStamped>(
        "itav_agv/tracked_pose",
        10,
        std::bind(&ITAVController::pose_callback, this, std::placeholders::_1));
    goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        10,
        std::bind(&ITAVController::goal_callback, this, std::placeholders::_1));
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("itav_agv/cmd_vel", 10);
    trajectory_pub = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    grid_map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "projected_map",
        10,
        std::bind(&ITAVController::grid_map_callback, this, std::placeholders::_1));
    obstacle_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        "obstacles_state",
        10,
        std::bind(&ITAVController::obstacle_callback, this, std::placeholders::_1));
    ob_pub = create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);

    // dwa = DWAPlanner(current_pose);
    // trajectory.poses.push_back(geometry_msgs::msg::PoseStamped());
}

void ITAVController::obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    auto obstacles = msg->data;
    std::vector<double> obs(obstacles.size());
    std::copy(obstacles.begin(), obstacles.end(), obs.begin());
    auto obstacles_matrix = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(obs.data(), obstacles.size() / 4, 4);
    if (current_pose.size() == 3) {
        double robot_x = current_pose[0];
        double robot_y = current_pose[1];
        obstacles_matrix = obstacles_matrix.rowwise().squaredNorm().eval().array() > 0.75 * 0.75;
        obstacles_matrix = (obstacles_matrix.col(0).array() + robot_x).matrix().transpose().finished().eval().comma(obstacles_matrix.col(1).array() + robot_y);
    }
    dwa.config.ob = obstacles_matrix;
}

void ITAVController::grid_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    grid.clear();
    auto map_data = msg->data;
    int map_width = msg->info.width;
    int map_height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    std::vector<int> map_row(map_width, 0);
    for (int i = 0; i < map_height; i++) {
        auto row_start = map_data.begin() + i * map_width;
        auto row_end = row_start + map_width
    std::vector<int> map_row(row_start, row_end);
    grid.push_back(map_row);
}

auto map_to_world = [&](const int& i, const int& j) {
    return std::make_pair(origin_x + j * resolution, origin_y + i * resolution);
};

std::vector<Eigen::VectorXd> obstacles;
for (int i = 0; i < grid.size(); i++) {
    for (int j = 0; j < grid[i].size(); j++) {
        if (grid[i][j] > 50) {
            auto world_pos = map_to_world(i, j);
            obstacles.push_back(Eigen::VectorXd::Map(world_pos.data(), world_pos.size()));
        }
    }
}

if (obstacles.size() > 0) {
    Eigen::MatrixXd obs(obstacles.size(), 4);
    for (int i = 0; i < obstacles.size(); i++) {
        obs.row(i) << obstacles[i](0), obstacles[i](1), 0, 0;
    }
    dwa.config.ob = obs;
}
}

void ITAVController::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
auto pose = msg->pose;
double x = pose.position.x;
double y = pose.position.y;
double _, , yaw;
tf2::Quaternion q;
tf2::fromMsg(pose.orientation, q);
tf2::Matrix3x3(q).getRPY(, _, yaw);
RCLCPP_INFO(this->get_logger(), "goal_pose received (%f, %f, %f)", x, y, yaw);
goal_pose = { x, y, yaw };
dwa.set_goal_pose(goal_pose);
}

void ITAVController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
auto pose = msg->pose.pose;
double x = pose.position.x;
double y = pose.position.y;
double _, , yaw;
tf2::Quaternion q;
tf2::fromMsg(pose.orientation, q);
tf2::Matrix3x3(q).getRPY(, _, yaw);
current_pose = { x, y, yaw };
dwa.set_current_pose(current_pose);
}

void ITAVController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
auto pose = msg->pose;
double x = pose.position.x;
double y = pose.position.y;
double _, , yaw;
tf2::Quaternion q;
tf2::fromMsg(pose.orientation, q);
tf2::Matrix3x3(q).getRPY(, _, yaw);
current_pose = { x, y, yaw };
dwa.set_current_pose(current_pose);
}

void ITAVController::controller()
{
if (goal_pose.size() == 3 && current_pose.size() == 3) {
auto u_and_predicted_trajectory = dwa.dwa_control();
auto u = u_and_predicted_trajectory.first;
auto predicted_trajectory = u_and_predicted_trajectory.second;
dwa.set_cmd_vel(u);

    trajectory.poses.clear();
    for (auto& pose : predicted_trajectory) {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = pose(0);
        p.pose.position.y = pose(1);
        p.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose(2));
        p.pose.orientation = tf2::toMsg(q);
        trajectory.poses.push_back(p);
    }
    trajectory.header.frame_id = "map";
    trajectory_pub->publish(trajectory);
    auto vel = geometry_msgs::msg::Twist();
    vel.linear.x = u(0);
    vel.angular.z = u(1);
    RCLCPP_INFO(this->get_logger(), "vel: (%f, %f)", vel.linear.x, vel.angular.z);
    RCLCPP_INFO(this->get_logger(), "goal_pose: (%f, %f, %f)", goal_pose[0], goal_pose[1], goal_pose[2]);
    publish_vel(vel);

    double dist_to_goal = std::hypot(goal_pose[0] - current_pose[0], goal_pose[1] - current_pose[1]);
    if (dist_to_goal < 0.5) {
        RCLCPP_INFO(this->get_logger(), "goal reached");
        goal_pose.clear();
        publish_vel(geometry_msgs::msg::Twist());
    }
}
else {
    RCLCPP_INFO(this->get_logger(), "waiting for goal_pose");
}
}

void ITAVController::plotter()
{
if (goal_pose.size() == 3 && current_pose.size() == 3) {
dwa.plot_all();
}
}

void ITAVController::publish_vel(const geometry_msgs::msg::Twist& vel)
{
cmd_vel_pub->publish(vel);
}

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<ITAVController>(rclcpp::NodeOptions());
RCLCPP_INFO(node->get_logger(), "waiting for goal_pose");
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}