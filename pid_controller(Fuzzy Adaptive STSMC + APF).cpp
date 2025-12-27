/**
 * *********************************************************
 * @file: pid_controller.cpp
 * @brief: Fuzzy Adaptive Super-Twisting SMC + APF Controller
 * @author: Wang Xuhui
 * @date: 2025-12-25
 * @version: 3.0
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "common/geometry/point.h"
#include "controller/pid_controller.h"

// 引入必要的库
#include <cmath>
#include <algorithm>
#include <vector>

PLUGINLIB_EXPORT_CLASS(rmp::controller::PIDController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {

PIDController::PIDController() : initialized_(false), goal_reached_(false), tf_(nullptr) {
}

PIDController::PIDController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : PIDController() {
  initialize(name, tf, costmap_ros);
}

PIDController::~PIDController() {
}

void PIDController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    pid_config_ = config_.pid_controller();

    // 初始化 PID 误差变量
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    // >>> 初始化 STSMC 积分项 <<<
    stsmc_integral_v_ = 0.0;

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    R_INFO << "Fuzzy STSMC-APF Controller Initialized!";
  } else {
    R_WARN << "PID Controller has already been initialized.";
  }
}

bool PIDController::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    R_ERROR << "This planner has not been initialized";
    return false;
  }

  R_INFO << "Got new plan";

  global_plan_.clear();
  global_plan_ = orig_global_plan;

  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y) {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
    
    // >>> 重置 STSMC 积分项，防止超调 <<<
    stsmc_integral_v_ = 0.0;
  }

  return true;
}

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

// =================================================================================
// 模糊推理部分 (Fuzzy Inference)
// =================================================================================

// 规则：误差越大 -> 增益越大
double PIDController::computeFuzzySMCGain(double error) {
    double abs_err = std::fabs(error);
    double k_min = 0.5;  // 最小扰动估计
    double k_max = 3.0;  // 最大扰动估计 (应对大风浪)
    
    // Sigmoid 隶属度函数
    double alpha = 5.0; 
    double center = 0.25; // 约 15 度
    
    double fuzzy_weight = 1.0 / (1.0 + std::exp(-alpha * (abs_err - center)));
    
    return k_min + (k_max - k_min) * fuzzy_weight;
}

// 规则：距离越近且角度越正 -> 斥力增益越大
double PIDController::computeFuzzyAPFGain(double dist, double angle) {
    double safe_dist = 1.0; 
    
    // 距离因子
    double mu_dist = 0.0;
    if (dist < safe_dist) {
        mu_dist = (safe_dist - dist) / safe_dist;
        mu_dist = std::pow(mu_dist, 1.5); 
    }

    // 角度因子 (高斯函数)
    double abs_ang = std::fabs(angle);
    double sigma = 0.6; 
    double mu_angle = std::exp(-(abs_ang * abs_ang) / (2 * sigma * sigma));

    // 模糊合成
    double fuzzy_output = mu_dist * mu_angle;

    double max_gain = 5.0; // 最大斥力强度
    return max_gain * fuzzy_output; 
}

// =================================================================================
// 主控制循环
// =================================================================================
bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "PID Controller has not been initialized";
    return false;
  }

  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, config_.map_frame(), robot_pose_odom, robot_pose_map);

  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_odom);

  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  
  // 自适应预瞄距离
  double L = clamp(std::fabs(vt) * pid_config_.lookahead_time(),
                   pid_config_.min_lookahead_dist(), pid_config_.max_lookahead_dist());

  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  double theta = tf2::getYaw(robot_pose_map.pose.orientation); 

  // === 阶段 A: 到达终点附近 ===
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    double e_theta = normalizeAngle(goal_theta_ - theta);

    if (!shouldRotateToPath(std::fabs(e_theta))) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
      stsmc_integral_v_ = 0.0; // 停止时重置积分
    } else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    }
  }
  // === 阶段 B: 正常行驶 (Fuzzy STSMC + APF) ===
  else {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);
    Eigen::Vector3d s_d(lookahead_pt.x(), lookahead_pt.y(), lookahead_pt.theta());
    Eigen::Vector2d u_r(vt, wt);

    // [Step 1]: 轨迹跟踪 (STSMC)
    Eigen::Vector2d u = pid_config_.model_based_mode() ?
                        _modelBasedPIDControl(s, s_d, u_r) :
                        _modelFreePIDControl(s, s_d, u_r); // 这里内部实现了 STSMC

    double v_final = u[0];
    double w_final = u[1];

    // [Step 2]: 局部避障 (Fuzzy APF)
    // ---------------------------------------------------------
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;
    
    // 将机器人世界坐标转为地图坐标
    if (costmap->worldToMap(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, mx, my)) {
        
        double min_dist = 100.0;
        double obstacle_yaw = 0.0;
        bool found_obstacle = false;
        
        // 搜索周围网格 (约 1米范围, 假设分辨率 0.05)
        int search_r = 20; 

        for (int i = -search_r; i <= search_r; ++i) {
            for (int j = -search_r; j <= search_r; ++j) {
                unsigned int nx = mx + i;
                unsigned int ny = my + j;
                
                if (nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) continue;
                
                unsigned char cost = costmap->getCost(nx, ny);
                // 仅处理高风险障碍物
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

        double safe_distance = 1.0; 
        if (found_obstacle && min_dist < safe_distance) {
            
            double relative_ang = normalizeAngle(obstacle_yaw - theta);
            
            // 视野范围：前方 120 度
            if (std::fabs(relative_ang) < (2.0 * M_PI / 3.0)) {
                
                // >>> 调用创新点：Fuzzy APF Gain <<<
                double repulsion_gain = computeFuzzyAPFGain(min_dist, relative_ang);
                
                // 斥力方向逻辑：障碍物在左(ang>0)，往右转(w<0)
                double direction = (relative_ang > 0) ? -1.0 : 1.0;
                
                // 势场公式：F = Gain * (1/d - 1/d0)
                double repulsion_w = repulsion_gain * (1.0/std::max(min_dist, 0.1) - 1.0/safe_distance) * direction;
                
                // 叠加斥力到角速度
                w_final += repulsion_w;
                
                // 动态减速：危险度越高，速度越慢
                // 归一化风险因子，用于衰减线速度
                double risk_factor = std::min(1.0, repulsion_gain / 5.0); 
                v_final *= (1.0 - risk_factor * 0.8); // 最多减速 80%
            }
        }
    }
    // ---------------------------------------------------------

    cmd_vel.linear.x = linearRegularization(vt, v_final);
    cmd_vel.angular.z = angularRegularization(wt, w_final);
  }

  // 可视化
  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();
  Points3d points;
  points.emplace_back(std::move(lookahead_pt));
  visualizer->publishPoints(points, target_pt_pub_, "map", "lookahead",
                            rmp::common::util::Visualizer::RED, 0.3);
  current_pose_pub_.publish(robot_pose_map);

  return true;
}

/**
 * @brief Execute Fuzzy Adaptive Super-Twisting SMC
 * @details 核心创新：使用模糊参数的二阶滑模算法，消除抖振并保证有限时间收敛
 */
Eigen::Vector2d PIDController::_modelFreePIDControl(Eigen::Vector3d s,
                                                    Eigen::Vector3d s_d,
                                                    Eigen::Vector2d u_r) {
  Eigen::Vector2d u;

  // 1. 计算航向误差
  double dx = s_d[0] - s[0];
  double dy = s_d[1] - s[1];
  double path_yaw = std::atan2(dy, dx);
  double e_theta = normalizeAngle(path_yaw - s[2]);
  
  // 滑模面 S 定义
  double S = e_theta;

  // 2. >>> 创新点：模糊增益估计 <<<
  // 根据当前误差大小，估计需要的扰动补偿强度
  double L_estimated = computeFuzzySMCGain(S);

  // 3. Super-Twisting 参数 (STSMC)
  // 理论公式：alpha = 1.5 * sqrt(L), beta = 1.1 * L
  double alpha = 1.5 * std::sqrt(L_estimated);
  double beta  = 1.1 * L_estimated;

  // 4. 计算符号项
  double sign_S = (S > 0) ? 1.0 : ((S < 0) ? -1.0 : 0.0);
  double sqrt_abs_S = std::sqrt(std::fabs(S));

  // 5. 更新积分项 (离散时间积分)
  // 假设控制周期 dt = 0.1s (或者从 costmap controller_frequency 获取)
  double dt = 0.1; 
  stsmc_integral_v_ += (-beta * sign_S) * dt;

  // 积分限幅 (Anti-windup)，防止数值爆炸
  double integral_limit = config_.max_angular_velocity(); 
  if (stsmc_integral_v_ > integral_limit) stsmc_integral_v_ = integral_limit;
  else if (stsmc_integral_v_ < -integral_limit) stsmc_integral_v_ = -integral_limit;

  // 6. 计算最终角速度控制律
  // u = -alpha * |S|^0.5 * sgn(S) + v
  double w_cmd = -alpha * sqrt_abs_S * sign_S + stsmc_integral_v_;

  // 7. 线速度控制律 (投影法)
  double v_cmd = config_.max_linear_velocity() * std::cos(e_theta);

  // 到达终点判定微调
  double dist_to_goal = std::hypot(dx, dy);
  if (dist_to_goal < 0.1) {
    v_cmd = 0.0;
    w_cmd = 0.0;
    stsmc_integral_v_ = 0.0;
  }
  
  // 8. 物理限幅
  if (v_cmd < 0.0) v_cmd = 0.0; 
  if (std::fabs(v_cmd) > config_.max_linear_velocity()) {
    v_cmd = std::copysign(config_.max_linear_velocity(), v_cmd);
  }
  if (std::fabs(w_cmd) > config_.max_angular_velocity()) {
    w_cmd = std::copysign(config_.max_angular_velocity(), w_cmd);
  }

  u[0] = v_cmd;
  u[1] = w_cmd;

  return u;
}

// 保持原有的基于模型的PID，用于对比或兼容
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