#include "pdwc/dwa.hpp"



DWA::DWA(Eigen::VectorXd current_pose) {
  x[0] = current_pose[0];
  x[1] = current_pose[1];
  x[2] = current_pose[2];
  x[3] = current_pose[3];
  x[4] = current_pose[4];
  
  goal[0] = 0.0;
  goal[1] = 0.0;
  u = {{0.0, 0.0}};

}

State DWA::motionModel(State x, Control u, float dt) {
  x[2] += u[1] * dt;
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
  return x;
};

void DWA::setObstacles(Eigen::MatrixXd ob) 
{
  this->config.ob = ob;
}

void DWA::setGoal(Point goal) 
{
  this->goal = goal;
}


void DWA::getFootprint(std::vector<std::array<float, 2>> &footprint)
{
  #pragma omp parallel for
  for (int i = 0; i < 4; i++)
  {
    footprint.push_back({config.footprint[i][0], config.footprint[i][1]});
  }
}
void DWA::setState(Eigen::VectorXd current_pose) {
  x[0] = current_pose[0];
  x[1] = current_pose[1];
  x[2] = current_pose[2];
  x[3] = current_pose[3];
  x[4] = current_pose[4];
}

void DWA::setFootprint(float footprint[][2])
{
  #pragma omp parallel for
  for (int i = 0; i < 4; i++)
  {
    config.footprint[i][0] = footprint[i][0];
    config.footprint[i][1] = footprint[i][1];
  }
}

void DWA::setMap(Eigen::MatrixXd map, double resolution, Eigen::Vector2d origin) {
  RCLCPP_INFO(rclcpp::get_logger("DWA"), "setMap");
  RCLCPP_INFO(rclcpp::get_logger("DWA"), "map size: %d, %d", map.rows(), map.cols());
  this->map_ = map;
  this -> resolution = resolution;
  this -> origin = origin;
}

void DWA::getTrajectories(std::vector<Traj> &trajectories) {
  trajectories = this->trajectories;
}


Window DWA::dynamicWindow(State x, Config config) {
  return {{
      std::max((x[3] - config.max_accel * config.dt), config.min_speed),
      std::min((x[3] + config.max_accel * config.dt), config.max_speed),
      std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
      std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)
  }};
};


Traj DWA::calc_trajectory(State x, float v, float w, Config config){
    Traj traj;
    traj.push_back(x);
    float time = 0.0;
    while (time <= config.predict_time){
        x = motionModel(x, std::array<float, 2>{{v, w}}, config.dt);
        traj.push_back(x);
        time += config.dt;
    }
    return traj;
}


float DWA::costObstacle(Traj traj, Config config){
  // calc obstacle cost inf: collision, 0:free
  int skip_n = 2;
  float min_dist = std::numeric_limits<float>::max();
  float sum_dist = 0; 
  float min_ttc = std::numeric_limits<float>::max();
  float dt = skip_n *config.dt;

  #pragma omp parallel for collapse(2)
  for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
    for (unsigned int i=0; i< config.ob.rows(); i++){
      double ox = config.ob.coeff(i,0);
      double oy = config.ob.coeff(i,1);
      double ovx = config.ob.coeff(i,2);
      double ovy = config.ob.coeff(i,3);

      // Distance-based cost calculation with footprint checking
      for (unsigned int j=0; j<sizeof(config.footprint[0]); j++){
        float px = traj[ii][0] + config.footprint[j][0] * std::cos(traj[ii][2]) - config.footprint[j][1] * std::sin(traj[ii][2]);
        float py = traj[ii][1] + config.footprint[j][0] * std::sin(traj[ii][2]) + config.footprint[j][1] * std::cos(traj[ii][2]);
        // ox+= ovx * config.dt;
        // oy+= ovy * config.dt;
        float dx = px - float(ox + ovx * dt * ii);
        float dy = py - float(oy + ovy * dt * ii);

        float dist = std::sqrt(dx * dx + dy * dy);
        sum_dist += dist;
        if (dist < min_dist) {
          min_dist = dist;
        }

      }
    }
  }

  // RCLCPP_INFO(rclcpp::get_logger("DWA"),
  //         "dist cost: %f, min_dist: %f", 1.0 / (min_dist + 1e-6), min_dist);
  // Check for collision
  if (min_dist <= config.obstacle_margin) {
  
    // RCLCPP_INFO(rclcpp::get_logger("DWA"), 
    //     "Collision course detected distance %f", min_dist);
    return config.collision_threshold / (min_dist + 1e-6);
  }
  // float dist_cost = 1.0 / (min_dist + 1e-6);
  float dist_cost = 1.0 / (min_dist + 1e-6);


  return dist_cost;// + ttc_cost;
};

float DWA::costGoal(Traj traj, Point goal, Config config){

  // float goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
  // float traj_magnitude = std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
  // float dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  // float error = dot_product / (goal_magnitude * traj_magnitude);
  // float error_angle = std::acos(error);
  // float cost =  error_angle;
  float dx = goal[0] - traj.back()[0];
  float dy = goal[1] - traj.back()[1];
  float dist = std::sqrt(dx * dx + dy * dy);
  // cost = dist;

  return dist;
};

Traj DWA::finalInput(
  State x, Control& u,
  Window dw, Config config, Point goal){

    float min_cost = 10000.0;
    Control min_u = u;
    min_u[0] = 0.0;
    Traj best_traj;
    trajectories.clear();
    // evalucate all trajectory with sampled input in dynamic window

    #pragma omp parallel for collapse(3)
    for (float v=dw[0]; v<=dw[1]; v+=config.v_reso){
      for (float w=dw[2]; w<=dw[3]; w+=config.yawrate_reso){

        Traj traj = (v > 0) ?  calc_trajectory(x, v, w, config) : calc_trajectory(x, v, -w, config);
        bool free = true;
        // check if trajectory is in occupied space of map_
        for (unsigned int i=0; i<traj.size(); i++){
          double x_ = (traj[i][0] - origin[0])/resolution;
          double y_ = (traj[i][1] - origin[1])/resolution;
          // make sure trajecotry is inside the map_
          if ((x_- 20) < 0 || map_.rows() <= (x_ + 20) || (y_ - 20) < 0 || map_.cols() <= (y_ + 20) || int(map_.coeff(int(x_), int(y_))) != 0)
            free = false;
        }
        if (free) {
          trajectories.push_back(traj);
          float to_goal_cost = config.to_goal_cost_gain * costGoal(traj, goal, config);

          float speed = traj.back()[3] > 0.0 ? traj.back()[3] : (traj.back()[3] * config.reverse_penality);
          float speed_cost = config.speed_cost_gain * (config.max_speed - speed);
          float ob_cost = config.obstacle_cost_gain * costObstacle(traj, config);
          float final_cost = to_goal_cost+ speed_cost + ob_cost;

          if (min_cost >= final_cost){
            min_cost = final_cost;
            min_u = Control{{v, w}};
            best_traj = traj;
          }
          }
      }
    }
    u = min_u;
    return best_traj;
};


Result DWA::DWAControl(){
    // # Dynamic Window control
    Window dw = dynamicWindow(x, config);
    Traj traj = finalInput(x, u, dw, config, goal);
    Result r;
    r = std::make_pair(u, traj);
    return r;
  }





// int main(){
//   State x({{0.0, 0.0, M_PI/8.0, 0.0, 0.0}});
//   Point goal({{10.0,10.0}});
//   Obstacle ob({
//     {{-1, -1}},
//     {{0, 2}},
//     {{4.0, 2.0}},
//     {{5.0, 4.0}},
//     {{5.0, 5.0}},
//     {{5.0, 6.0}},
//     {{5.0, 9.0}},
//     {{8.0, 9.0}},
//     {{7.0, 9.0}},
//     {{12.0, 12.0}}
//   });

//   Control u({{0.0, 0.0}});
//   Config config;
//   Traj traj;
//   traj.push_back(x);

//   bool terminal = false;

//   int count = 0;
//   // DWA planner;
//   // for(int i=0; i<1000 && !terminal; i++){
//   //   Traj ltraj = planner.DWAControl(x, u, config, goal, ob);
//   //   x = planner.motionModel(x, u, config.dt);
//   //   traj.push_back(x);

//   //   count++;
//   // }
// }