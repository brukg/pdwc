#include <rclcpp/rclcpp.hpp>
#include <pdwa/pdwa.hpp>

ITAVController::ITAVController(const rclcpp::NodeOptions& options) : Node("dwa_controller")
{

    // Initialize DWA
    current_pose = Eigen::VectorXd::Zero(5);
    dwa = std::make_shared<DWA>(current_pose);
    RCLCPP_INFO(this->get_logger(), "DWA initialized");

    RCLCPP_INFO(this->get_logger(), "Controller initialized");
    declare_parameter<int>("rate", 100.0);
    declare_parameter<std::string>("footprint" , "[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]");
    declare_parameter<float>("max_speed", 0.5);
    declare_parameter<float>("min_speed", -0.5);
    declare_parameter<float>("max_yawrate", 1.0);
    declare_parameter<float>("max_accel", 0.2);
    declare_parameter<float>("max_dyawrate", 10.0);
    declare_parameter<float>("v_reso", 0.1);
    declare_parameter<float>("yawrate_reso", 0.1);
    declare_parameter<float>("dt", 0.1);
    declare_parameter<float>("predict_time", 1.0);
    declare_parameter<float>("to_goal_cost_gain", 0.1);
    declare_parameter<float>("speed_cost_gain", 0.1);
    declare_parameter<float>("obstacle_cost_gain", 0.1);
    declare_parameter<float>("reverse_penality", 2.0);
    declare_parameter<float>("robot_radius", 0.5);
    declare_parameter<float>("obstacle_radius", 0.5);
    declare_parameter<float>("obstacle_margin", 0.5);
    declare_parameter<float>("collision_threshold", 0.5);
    declare_parameter<float>("goal_tolerance", 0.5);


    // Get parameters
    get_parameter("rate", rate);
    get_parameter("footprint", footprint_string);
    get_parameter("max_speed", dwa->config.max_speed);
    get_parameter("min_speed", dwa->config.min_speed);
    get_parameter("max_yawrate", dwa->config.max_yawrate);
    get_parameter("max_accel", dwa->config.max_accel);
    get_parameter("max_dyawrate", dwa->config.max_dyawrate);
    get_parameter("v_reso", dwa->config.v_reso);
    get_parameter("yawrate_reso", dwa->config.yawrate_reso);  
    get_parameter("dt", dwa->config.dt);
    get_parameter("predict_time", dwa->config.predict_time);
    get_parameter("to_goal_cost_gain", dwa->config.to_goal_cost_gain);
    get_parameter("speed_cost_gain", dwa->config.speed_cost_gain);
    get_parameter("obstacle_cost_gain", dwa->config.obstacle_cost_gain);
    get_parameter("reverse_penality", dwa->config.reverse_penality);
    get_parameter("robot_radius", dwa->config.robot_radius);
    get_parameter("obstacle_radius", dwa->config.obstacle_radius);
    get_parameter("obstacle_margin", dwa->config.obstacle_margin);
    get_parameter("collision_threshold", dwa->config.collision_threshold);
    get_parameter("goal_tolerance", dwa->config.goal_tolerance);

    // map the footprint to footprint[4][2] of points
    std::istringstream iss(footprint_string);

    char c1, c2, comma;
    float num;
    float result[4][2];
    char ch;

    for (int i = 0; i < 4; ++i) {
        iss >> ch; // Read '[' or ','
        iss >> ch; // Read '['
        iss >> result[i][0];
        iss >> ch; // Read ','
        iss >> result[i][1];
        iss >> ch; // Read ']'
    }
    dwa->setFootprint(result);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&ITAVController::odomCallback, this, std::placeholders::_1));
    tracked_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "global_pose", 10, std::bind(&ITAVController::poseCallback, this, std::placeholders::_1));
    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&ITAVController::goalCallback, this, std::placeholders::_1));
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    trajectory_pub = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    grid_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1, std::bind(&ITAVController::gridMapCallback, this, std::placeholders::_1));
    obstacle_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "tracker/obstacles_state", 10, std::bind(&ITAVController::obstacleCallback, this, std::placeholders::_1));

    grid_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    footprint_pub = this->create_publisher<visualization_msgs::msg::Marker>("controller/footprint", 10);
    respones_time_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("controller/response_time", 10);
    // ~~~~~~~~~~~~~
    // ob_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
    RCLCPP_INFO(this->get_logger(), "Subscribers initialized");

    RCLCPP_INFO(this->get_logger(), "DWA Parameters:\n max_speed: %f\n min_speed: %f\n  max_yawrate: %f\n" 
                                    "max_accel: %f\n max_dyawrate: %f\n v_reso: %f\n yawrate_reso: %f\n dt: %f\n" 
                                    "predict_time: %f \n to_goal_cost_gain: %f \n speed_cost_gain: %f\n "
                                    "obstacle_cost_gain: %f \n robot_radius: %f \n obstacle_radius: %f \n "
                                    "collision_threshold: %f\n"
                                    "reverse_penality: %f\n"
                                    "obstacle_margin: %f\n"
                                    "goal_tolerance: %f\n",

                    dwa->config.max_speed, dwa->config.min_speed, dwa->config.max_yawrate, dwa->config.max_accel,
                    dwa->config.max_dyawrate, dwa->config.v_reso, dwa->config.yawrate_reso, dwa->config.dt,
                    dwa->config.predict_time, dwa->config.to_goal_cost_gain, dwa->config.speed_cost_gain,
                    dwa->config.obstacle_cost_gain, dwa->config.robot_radius, dwa->config.obstacle_radius,
                    dwa->config.collision_threshold, dwa->config.reverse_penality, dwa->config.obstacle_margin, dwa->config.goal_tolerance);

    // Get map service call
    get_map_client = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Start trajectory service call
    start_trajectory_client = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("/start_trajectory");

    // List of trajectories publisher
    trajectories_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_list", 10);

    RCLCPP_INFO(this->get_logger(), "Services initialized");
    trajectory = Eigen::ArrayXXd::Zero(1, 3);

    // Obstacles
    dwa->config.ob = Eigen::ArrayXXd(1, 4);
    dwa->config.ob << 20, 20, 0, 0.2;
    int counter = 0;
    while (!get_map_client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Map service not available, waiting again...");
      if (counter++ > 5){
        rclcpp::shutdown();
        exit(0);
      }
    }

    auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = get_map_client->async_send_request(req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    RCLCPP_INFO(this->get_logger(), "Map service available");
    auto map_response = future.get();

    if (map_response != nullptr)
    {
      auto map_data = map_response->map.data;
      map_width = map_response->map.info.width;
      map_height = map_response->map.info.height;
      map_resolution = map_response->map.info.resolution;
      map_origin = Eigen::Vector2d(map_response->map.info.origin.position.x, map_response->map.info.origin.position.y);

      // // Convert the map_data to an Eigen Matrix
      // Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> grid_map_int(reinterpret_cast<int*>(map_data.data()), map_width, map_height);
      // grid << grid_map_int.cast<double>();
      grid = Eigen::MatrixXd(map_height, map_width);
      for (int i = 0; i < map_height; ++i) {
          for (int j = 0; j < map_width; ++j) {
            grid(i, j) = static_cast<double>(map_data[i * map_width + j]);
            // RCLCPP_INFO(this->get_logger(), "Map received value at %d, %d: %f", i, j, cell);
            // RCLCPP_INFO(this->get_logger(), "Map received value at %d, %d: %f", i, j, grid(i, j));

          }
      }
      // shift the map according to the map map_origin
      
      dwa->setMap(grid.transpose(), map_resolution, map_origin);
      // dwa->setMap(grid, map_resolution, map_origin);
      RCLCPP_INFO(this->get_logger(), "Map received value grid dimensions: %d, %d", grid.rows(), grid.cols());

      // RCLCPP_INFO(this->get_logger(), "Map received value at 10, 10: %d", int(grid.transpose().coeff(int((23.99620 - map_origin[0])/map_resolution), int((53.87680 - map_origin[1])/map_resolution)))!=0);
      // RCLCPP_INFO(this->get_logger(), "Map received value at 26, 53: %d", (grid.transpose().coeff(int((26.0 - map_origin[0])/map_resolution), int((53 - map_origin[1])/map_resolution)))==0);
      // RCLCPP_INFO(this->get_logger(), "Map received value at 0, -4: %f", grid.transpose().coeff(int((0 - map_origin[0])/map_resolution), int((-4.0 - map_origin[1])/map_resolution)));
      // RCLCPP_INFO(this->get_logger(), "Map received value at 2.5, 3: %d", int(grid.transpose().coeff(int((2.5 - map_origin[0])/map_resolution), int((3.0 - map_origin[1])/map_resolution)))!=0);
      // RCLCPP_INFO(this->get_logger(), "Map received value at 2, 3: %f", grid.transpose().coeff(int((2 - map_origin[0])/map_resolution), int((3.0 - map_origin[1])/map_resolution)));
    }
    
    dwa -> getFootprint(footprint);
    goal_pose = Eigen::VectorXd::Zero(3);


    // Timer for the controller
    timer = this->create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&ITAVController::controller, this));
}

void ITAVController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Process the data
    // current_pose[0] = msg->pose.pose.position.x;
    // current_pose[1] = msg->pose.pose.position.y;
    // double yaw = atan2(2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
    //                    1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));
    // current_pose[2] = yaw;
    current_pose[3] = msg->twist.twist.linear.x;
    current_pose[4] = msg->twist.twist.angular.z;
    dwa-> setState(current_pose);
}

void ITAVController::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
{
  // RCLCPP_INFO(this->get_logger(), "Pose received");
  current_pose[0] = msg->pose.pose.position.x;
  current_pose[1] = msg->pose.pose.position.y;
  double yaw = atan2(2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
                      1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));
  current_pose[2] = yaw;

  visualization_msgs::msg::Marker footprint_marker;
  footprint_marker.header.frame_id = "map";
  footprint_marker.header.stamp = this->now();
  footprint_marker.ns = "footprint";
  footprint_marker.id = 0;
  footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  footprint_marker.action = visualization_msgs::msg::Marker::ADD;
  footprint_marker.pose.orientation.x = 0.0;
  footprint_marker.pose.orientation.y = 0.0;
  footprint_marker.pose.orientation.z = 0.0;
  footprint_marker.pose.orientation.w = 1.0;
  footprint_marker.scale.x = 0.05;
  footprint_marker.color.r = 1.0;
  footprint_marker.color.a = 1.0;
  footprint_marker.points.resize(footprint.size() + 1);
  // transform footprint to map frame
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = current_pose[0] + footprint[i][0] * cos(current_pose[2]) - footprint[i][1] * sin(current_pose[2]);
    p.y = current_pose[1] + footprint[i][0] * sin(current_pose[2]) + footprint[i][1] * cos(current_pose[2]);
    p.z = 0.0;
    footprint_marker.points[i] = p;
  }
  footprint_marker.points[footprint.size()] = footprint_marker.points[0];



  footprint_pub->publish(footprint_marker);

}
void ITAVController::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
  // RCLCPP_INFO(this->get_logger(), "Goal received");
  goal_pose [0] = msg->pose.position.x;
  goal_pose [1] = msg->pose.position.y;
  double yaw = atan2(2 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
                      1 - 2 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z));
  goal_pose [2] = yaw;
  RCLCPP_INFO(this->get_logger(), "Goal received: %f, %f, %f", goal_pose[0], goal_pose[1], goal_pose[2]);
  dwa-> setGoal(goal_pose);

}

void ITAVController::obstacleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // first remove the obstacle that is the same as the current robot pose by checking the distance
    auto msg_data = msg->data;
    int size_ = msg_data.size();
    nav_msgs::msg::OccupancyGrid grid_map;
    grid_map.header.frame_id = "map";
    grid_map.info.resolution = map_resolution;
    grid_map.info.width = map_width;
    grid_map.info.height = map_height;
    grid_map.info.origin.position.x = map_origin[0];
    grid_map.info.origin.position.y = map_origin[1];
    grid_map.info.origin.position.z = 0.0;
    grid_map.data.resize(map_width * map_height);

    // RCLCPP_INFO(this->get_logger(), "Obstacles received size: %d", size_/4);

    if (size_ == 0 || grid.rows() == 0) {
      obstacles = Eigen::MatrixXd(2, 4);
      obstacles << 8000, 8000, 0, 0.0,
                   8000, 8000, 0, 0.0;
    } else {
        // populate the obstacles matrix
        obstacles = Eigen::MatrixXd(int(size_ / 4), 4);

        for (int i = 0; i < int(size_ / 4); i++) {
        // RCLCPP_INFO(this->get_logger(), "msg data: %f, %f,", msg_data[4*i], msg_data[4*i + 1]);        
          // add obstacle to the map
          // int x = int((obstacles(i, 0) - map_origin[0]) / map_resolution);
          // int y = int((obstacles(i, 1) - map_origin[1]) / map_resolution);
          // grid_map.data[x * map_width + y] = 0;
          
          // remove the obstacles that are too close to the robot since it is the robot itself
          bool in_robot_ = (sqrt(pow(msg_data[4*i] - current_pose[0], 2) + pow(msg_data[4*i + 1] - current_pose[1], 2)) < 1.5);
          // RCLCPP_INFO(this->get_logger(), "robot pose: %f, in robot: %d", current_pose[0], in_robot_);
          // check if the obstacle is already in the map and remove it
          int x = int((msg_data[4*i] - map_origin[0]) / map_resolution);
          int y = int((msg_data[4*i + 1] - map_origin[1]) / map_resolution);
          bool in_map_ = (x>=0 && y>=0) ? (grid.transpose().coeff(x, y) > 0): false;
          // RCLCPP_INFO(this->get_logger(), "in map: %d", in_map_);
          if (in_robot_ || in_map_ )  { 
            // if (in_robot_) RCLCPP_INFO(this->get_logger(), "removed internal obstacle %f, %f", msg_data[4*i], msg_data[4*i + 1]);
            // if (in_map_) RCLCPP_INFO(this->get_logger(), "removed map obstacle %f, %f", msg_data[4*i], msg_data[4*i + 1]);
            obstacles(i, 0) = -1;
            obstacles(i, 1) = -1;
            obstacles(i, 2) = -1;
            obstacles(i, 3) = -1;

            continue;
          }
         
          // rotate the obstacle to the robot frame
          // double x = msg_data[4*i], y = msg_data[4*i + 1], vx = msg_data[4*i + 2], vy = msg_data[4*i + 3];
          obstacles(i, 0) = msg_data[4*i];
          obstacles(i, 1) = msg_data[4*i + 1];
          obstacles(i, 2) = msg_data[4*i + 2];
          obstacles(i, 3) = msg_data[4*i + 3];

        }
       
    }
    
    // grid_map_pub -> publish(grid_map);
    // get the type of the msg_data
    // RCLCPP_INFO(this->get_logger(), "Obstacle received: %f, %f, %f, %f", obstacles.coeff(0, 0), obstacles.coeff(0, 1), obstacles.coeff(0, 2), obstacles.coeff(0, 3));

    dwa->setObstacles(obstacles);
}

void ITAVController::gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Process the grid map

    // dwa->setMap(grid);
}
void ITAVController::controller()
{   
  // RCLCPP_INFO(this->get_logger(), "Controller");
  // RCLCPP_INFO(this->get_logger(), "Current pose: %f, %f, %f", current_pose[0], current_pose[1], current_pose[2]);
  geometry_msgs::msg::Twist cmd_vel;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  std::vector<Traj> traj;
  visualization_msgs::msg::MarkerArray trajectories;
  // if goal is set then run the controller
  if (goal_pose[0] != 0 && goal_pose[1] != 0 && current_pose[0] != 0 && current_pose[1] != 0)
  {
    // RCLCPP_INFO(this->get_logger(), "Goal pose: %f, %f, %f", goal_pose[0], goal_pose[1], goal_pose[2]);
    // start counting time
    auto start = std::chrono::high_resolution_clock::now();
    Result result = dwa->DWAControl();
    auto finish = std::chrono::high_resolution_clock::now();
    cmd_vel.linear.x = result.first[0];
    cmd_vel.angular.z = result.first[1];
    cmd_vel_pub->publish(cmd_vel);
    std::chrono::duration<double> elapsed = finish - start;
    std_msgs::msg::Float32MultiArray time_taken;
    time_taken.data.push_back(elapsed.count() * 1000);
    respones_time_pub->publish(time_taken);
    
    RCLCPP_INFO(this->get_logger(), "Time taken in milliseconds: %f", elapsed.count() * 1000);
    // RCLCPP_INFO(this->get_logger(), "v: %f, w: %f", result.first[0], result.first[1]);
    // RCLCPP_INFO(this->get_logger(), "Trajectory data size: %d", result.second.size());
    path.header.frame_id = "map";
    for (int i = 0; i < result.second.size(); i++)
    { 
      pose.header.frame_id = "map";
      pose.pose.position.x = result.second[i][0];
      pose.pose.position.y = result.second[i][1];
      pose.pose.position.z = 0;
      path.poses.push_back(pose);
    }
    trajectory_pub->publish(path);
    // cmd_vel.linear.x = result.v;
    dwa->getTrajectories(traj);

    trajectories.markers.resize(traj.size());
    for (int i = 0; i < traj.size(); i++)
    {
      trajectories.markers[i].header.frame_id = "map";
      trajectories.markers[i].header.stamp = this->now();
      trajectories.markers[i].ns = "traj";
      trajectories.markers[i].id = i;
      trajectories.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
      trajectories.markers[i].action = visualization_msgs::msg::Marker::ADD;
      trajectories.markers[i].pose.orientation.w = 1.0;
      trajectories.markers[i].scale.x = 0.05;
      trajectories.markers[i].color.a = 1.0;
      trajectories.markers[i].color.r = 0.0;
      trajectories.markers[i].color.g = 0.5;
      trajectories.markers[i].color.b = 0.0;
      trajectories.markers[i].points.resize(traj[i].size());
      for (int j = 0; j < traj[i].size(); j++)
      {
        trajectories.markers[i].points[j].x = traj[i][j][0];
        trajectories.markers[i].points[j].y = traj[i][j][1];
        trajectories.markers[i].points[j].z = 0;
      }
    }
    trajectories_pub->publish(trajectories);
    // if goal is reached then stop the robot
    if (sqrt(pow(current_pose[0] - goal_pose[0], 2) + pow(current_pose[1] - goal_pose[1], 2)) < dwa->config.goal_tolerance)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      cmd_vel_pub->publish(cmd_vel);
      goal_pose[0] = 0;
      goal_pose[1] = 0;
    }

  } //else RCLCPP_INFO(this->get_logger(), "Goal not set");

}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ITAVController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
