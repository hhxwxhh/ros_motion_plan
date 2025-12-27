/**
 * *********************************************************
 * @file: pid_controller.cpp
 * @brief: Robust Controller combining SMC (for tracking) and APF (for avoidance)
 * @author: Wang Xuhui
 * @date: 2025-12-25
 * @version: 3.0 (SMC + APF Fusion)
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "common/geometry/point.h"
#include "controller/pid_controller.h"

// 引入必要的标准库
#include <cmath>
#include <algorithm>
#include <vector>

PLUGINLIB_EXPORT_CLASS(rmp::controller::PIDController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {

/**
 * @brief Construct a new PIDController object
 */
PIDController::PIDController() : initialized_(false), goal_reached_(false), tf_(nullptr) {
}

/**
 * @brief Construct a new PIDController object
 */
PIDController::PIDController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : PIDController() {
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the PIDController object
 */
PIDController::~PIDController() {
}

/**
 * @brief Initialization of the local planner
 */
void PIDController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    pid_config_ = config_.pid_controller();

    // 依然初始化这些变量防止潜在报错
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    R_INFO << "SMC-APF Fusion Controller Initialized! (Robustness + Obstacle Avoidance)";
  } else {
    R_WARN << "PID Controller has already been initialized.";
  }
}

/**
 * @brief Set the plan that the controller is following
 */
bool PIDController::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    R_ERROR << "This planner has not been initialized";
    return false;
  }

  R_INFO << "Got new plan";

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // receive a plan for a new goal
  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y) {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
  }

  return true;
}

/**
 * @brief Check if the goal pose has been achieved
 */
bool PIDController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "PID Controller has not been initialized";
    return false;
  }

  if (goal_reached_) {
    R_INFO << "GOAL Reached!";
    return true;
  }
  return false;
}

/**
 * @brief Given the current position, compute the velocity commands
 * @note  这里集成了 APF 避障逻辑
 */
bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "PID Controller has not been initialized";
    return false;
  }

  // 1. 获取里程计信息
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // 2. 获取机器人位置
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, config_.map_frame(), robot_pose_odom, robot_pose_map);

  // 3. 剪枝全局路径
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_odom);

  // 4. 计算预瞄距离 (Look-ahead Distance)
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = clamp(std::fabs(vt) * pid_config_.lookahead_time(),
                   pid_config_.min_lookahead_dist(), pid_config_.max_lookahead_dist());

  // 5. 获取预瞄点
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  // 获取当前航向角
  double theta = tf2::getYaw(robot_pose_map.pose.orientation); 

  // === 阶段 A: 到达终点附近的逻辑 (保持不变) ===
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    double e_theta = normalizeAngle(goal_theta_ - theta);

    if (!shouldRotateToPath(std::fabs(e_theta))) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    }
  }
  // === 阶段 B: 正常行驶 (融合 SMC + APF) ===
  else {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);
    Eigen::Vector3d s_d(lookahead_pt.x(), lookahead_pt.y(), lookahead_pt.theta());
    Eigen::Vector2d u_r(vt, wt);

    // [Step 1]: 调用 SMC 算法获取基础控制量 (对应原来的 _modelFreePIDControl)
    Eigen::Vector2d u = pid_config_.model_based_mode() ?
                        _modelBasedPIDControl(s, s_d, u_r) :
                        _modelFreePIDControl(s, s_d, u_r);

    double v_final = u[0];
    double w_final = u[1];

    // [Step 2]: APF (人工势场) 局部避障逻辑 - 直接嵌入在这里
    // ---------------------------------------------------------
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;
    
    // 如果能获取到机器人在地图格子的坐标
    if (costmap->worldToMap(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, mx, my)) {
        
        double min_dist = 100.0;     // 最近障碍物距离
        double obstacle_yaw = 0.0;   // 最近障碍物方向
        bool found_obstacle = false;
        
        // 搜索半径 (例如 20 个格子, 约 1.0m)
        int search_r = 20; 

        for (int i = -search_r; i <= search_r; ++i) {
            for (int j = -search_r; j <= search_r; ++j) {
                unsigned int nx = mx + i;
                unsigned int ny = my + j;
                
                // 边界检查
                if (nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) continue;
                
                unsigned char cost = costmap->getCost(nx, ny);
                
                // LETHAL_OBSTACLE = 254, INSCRIBED_INFLATED_OBSTACLE = 253
                // 我们检测大于 128 的都视为危险物
                if (cost > 128) {
                    double wx, wy;
                    costmap->mapToWorld(nx, ny, wx, wy);
                    double dist = std::hypot(wx - robot_pose_map.pose.position.x, 
                                           wy - robot_pose_map.pose.position.y);
                    
                    if (dist < min_dist) {
                        min_dist = dist;
                        obstacle_yaw = std::atan2(wy - robot_pose_map.pose.position.y, 
                                                wx - robot_pose_map.pose.position.x);
                        found_obstacle = true;
                    }
                }
            }
        }

        // 如果发现了危险距离内的障碍物 (例如 < 0.6m)
        double safe_distance = 0.6;
        if (found_obstacle && min_dist < safe_distance) {
            // 计算障碍物相对于车头的角度
            double relative_ang = normalizeAngle(obstacle_yaw - theta);
            
            // 仅当前方有障碍物时才避让 (|angle| < 90度)
            if (std::fabs(relative_ang) < M_PI_2) {
                
                // 1. 斥力产生角速度：障碍物在左，往右转；障碍物在右，往左转
                // 距离越近，斥力越大
                double repulsion_gain = 2.0; 
                double repulsion_w = -1.0 * repulsion_gain * (1.0/min_dist - 1.0/safe_distance) * (relative_ang > 0 ? 1.0 : -1.0);
                
                // 叠加斥力
                w_final += repulsion_w;
                
                // 2. 减速机制：越近越慢
                double speed_factor = std::max(0.0, (min_dist - 0.25) / (safe_distance - 0.25));
                v_final *= speed_factor;
                
                // 3. 紧急制动
                if (min_dist < 0.3) {
                    v_final = 0.0;
                    // 原地旋转以摆脱困境
                    w_final = (relative_ang > 0) ? -0.5 : 0.5;
                }
            }
        }
    }
    // ---------------------------------------------------------

    // [Step 3]: 最终限幅与输出
    cmd_vel.linear.x = linearRegularization(vt, v_final);
    cmd_vel.angular.z = angularRegularization(wt, w_final);
  }

  // 可视化
  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();

  // publish lookahead pose
  Points3d points;
  points.emplace_back(std::move(lookahead_pt));
  visualizer->publishPoints(points, target_pt_pub_, "map", "lookahead",
                            rmp::common::util::Visualizer::RED, 0.3);

  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);

  return true;
}

/**
 * @brief Execute Sliding Mode Control (SMC)
 * @note  This replaces the original PID logic with SMC logic for better robustness
 */
Eigen::Vector2d PIDController::_modelFreePIDControl(Eigen::Vector3d s,
                                                    Eigen::Vector3d s_d,
                                                    Eigen::Vector2d u_r) {
  Eigen::Vector2d u;

  // 1. 计算误差
  double dx = s_d[0] - s[0];
  double dy = s_d[1] - s[1];
  double path_yaw = std::atan2(dy, dx);
  
  // 航向误差
  double e_theta = normalizeAngle(path_yaw - s[2]);

  // 2. SMC 参数 (可调)
  // k_approach: 趋近速度
  // k_switch:   抗扰强度 (Switching Gain)
  const double k_approach = 2.0; 
  const double k_switch   = 1.0;   
  const double boundary   = 0.15; // 饱和边界，防抖振

  // 3. 计算 Sat(S)
  double S = e_theta;
  double sat_S = 0.0;
  
  if (S > boundary) sat_S = 1.0;
  else if (S < -boundary) sat_S = -1.0;
  else sat_S = S / boundary;

  // 4. SMC 角速度控制律
  double w_cmd = k_approach * S + k_switch * sat_S;

  // 5. 线速度控制律 (结合投影法)
  // 注意：这里修复了之前的 max_linear_velocity 作用域问题
  // 使用 config_ 而不是 pid_config_
  double v_cmd = config_.max_linear_velocity() * std::cos(e_theta);

  // 到达终点附近的简单处理
  double dist_to_goal = std::hypot(dx, dy);
  if (dist_to_goal < 0.1) {
    v_cmd = 0.0;
    w_cmd = 0.0;
  }
  
  // 6. 安全限幅
  if (v_cmd < 0.0) v_cmd = 0.0; 
  
  // 线速度限幅
  if (std::fabs(v_cmd) > config_.max_linear_velocity()) {
    v_cmd = std::copysign(config_.max_linear_velocity(), v_cmd);
  }

  // 角速度限幅
  if (std::fabs(w_cmd) > config_.max_angular_velocity()) {
    w_cmd = std::copysign(config_.max_angular_velocity(), w_cmd);
  } 
  // 保持最小转速以克服摩擦 (可选)
  else if (std::fabs(w_cmd) < config_.min_angular_velocity() && std::fabs(w_cmd) > 0.01) {
    w_cmd = std::copysign(config_.min_angular_velocity(), w_cmd);
  }

  u[0] = v_cmd;
  u[1] = w_cmd;

  return u;
}

/**
 * @brief Execute model-based PID control process
 * @note  Keep original implementation for compatibility
 */
Eigen::Vector2d PIDController::_modelBasedPIDControl(Eigen::Vector3d s,
                                                     Eigen::Vector3d s_d,
                                                     Eigen::Vector2d u_r) {
  Eigen::Vector2d u;
  Eigen::Vector3d e(s_d - s);
  Eigen::Vector2d sx_dot(pid_config_.k_feedback() * e[0],
                         pid_config_.k_feedback() * e[1]);
  Eigen::Matrix2d R_inv;
  R_inv << std::cos(s[2]), sin(s[2]),
      -std::sin(s[2]) / pid_config_.dist_from_center_to_front_edge(),
      std::cos(s[2]) / pid_config_.dist_from_center_to_front_edge();
  u = R_inv * sx_dot;

  return u;
}
}  // namespace controller
}  // namespace rmp