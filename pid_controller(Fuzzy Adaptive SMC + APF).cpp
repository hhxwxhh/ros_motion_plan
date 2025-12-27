/**
 * *********************************************************
 * @file: pid_controller.cpp
 * @brief: Fuzzy Adaptive Robust Controller (SMC + APF)
 * @author: Wang Xuhui
 * @date: 2025-12-25
 * @version: 4.0 (Fuzzy Adaptive SMC + APF)
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "common/geometry/point.h"
#include "controller/pid_controller.h"

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

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    R_INFO << "Fuzzy Adaptive SMC-APF Controller Initialized!";
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
// 核心创新点 1: 模糊规则计算 SMC 增益 (实现简单的模糊推理机)
// 规则：误差越大 -> 增益越大 (快速收敛)；误差越小 -> 增益越小 (柔性抗抖振)
// =================================================================================
double PIDController::computeFuzzySMCGain(double error) {
    double abs_err = std::fabs(error);
    double k_min = 1.0;  // 最小增益 (保持基本响应)
    double k_max = 5.0;  // 最大增益 (对抗大扰动)
    
    // 隶属度函数 (Membership Function) 近似
    // 使用 Sigmoid 函数平滑过渡，代替复杂的 IF-THEN 查表
    // 形状：S型曲线，误差在 0.1~0.5 之间从 k_min 过渡到 k_max
    
    // alpha 决定了曲线的陡峭程度
    double alpha = 5.0; 
    // center 决定了突变的阈值中心 (0.25弧度 约 15度)
    double center = 0.25; 
    
    double fuzzy_weight = 1.0 / (1.0 + std::exp(-alpha * (abs_err - center)));
    
    // 归一化输出
    double k_adaptive = k_min + (k_max - k_min) * fuzzy_weight;
    
    return k_adaptive;
}

// =================================================================================
// 核心创新点 2: 模糊规则计算 APF 斥力增益
// 规则：
// 1. 距离越近 (CLOSE) -> 斥力大
// 2. 相对角度越小 (FRONT) -> 斥力大 (迎头相撞最危险)
// =================================================================================
double PIDController::computeFuzzyAPFGain(double dist, double angle) {
    // 安全距离参考
    double safe_dist = 0.8; 
    
    // 1. 距离因子 (Distance Factor): 越近越大 [0, 1]
    double mu_dist = 0.0;
    if (dist < safe_dist) {
        mu_dist = (safe_dist - dist) / safe_dist; // 线性增加
        mu_dist = std::pow(mu_dist, 1.5);         // 指数加强，越近反应越剧烈
    }

    // 2. 角度因子 (Angle Factor): 正对时(0度)最大 [0, 1]
    // 使用高斯函数形状: exp(-x^2)
    double abs_ang = std::fabs(angle);
    double sigma = 0.5; // 角度敏感度
    double mu_angle = std::exp(-(abs_ang * abs_ang) / (2 * sigma * sigma));

    // 3. 模糊推理合成 (Product Inference Engine)
    // 只有当 "近" AND "正对" 时，增益才最大
    double fuzzy_output = mu_dist * mu_angle;

    // 映射到实际物理增益
    double max_gain = 4.0;
    return max_gain * fuzzy_output; 
}

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
  
  // 自适应调整预瞄距离 (也可以加入模糊逻辑，但这里保持原样以防过于复杂)
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
    } else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    }
  }
  // === 阶段 B: 正常行驶 (Fuzzy Adaptive SMC + APF) ===
  else {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);
    Eigen::Vector3d s_d(lookahead_pt.x(), lookahead_pt.y(), lookahead_pt.theta());
    Eigen::Vector2d u_r(vt, wt);

    // [Step 1]: 调用 自适应 SMC 算法
    Eigen::Vector2d u = pid_config_.model_based_mode() ?
                        _modelBasedPIDControl(s, s_d, u_r) :
                        _modelFreePIDControl(s, s_d, u_r); // 内部已修改为使用模糊增益

    double v_final = u[0];
    double w_final = u[1];

    // [Step 2]: Fuzzy APF (模糊自适应势场避障)
    // ---------------------------------------------------------
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;
    
    if (costmap->worldToMap(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, mx, my)) {
        
        double min_dist = 100.0;
        double obstacle_yaw = 0.0;
        bool found_obstacle = false;
        
        int search_r = 20; 

        for (int i = -search_r; i <= search_r; ++i) {
            for (int j = -search_r; j <= search_r; ++j) {
                unsigned int nx = mx + i;
                unsigned int ny = my + j;
                
                if (nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) continue;
                
                unsigned char cost = costmap->getCost(nx, ny);
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

        double safe_distance = 0.8; // 稍微增大一点探测范围，给模糊算法发挥空间
        if (found_obstacle && min_dist < safe_distance) {
            
            double relative_ang = normalizeAngle(obstacle_yaw - theta);
            
            // 只要在前方 120 度范围内都考虑 (扩大视野)
            if (std::fabs(relative_ang) < (2.0 * M_PI / 3.0)) {
                
                // >>> 创新点调用：使用模糊逻辑计算斥力增益 <<<
                double repulsion_gain = computeFuzzyAPFGain(min_dist, relative_ang);
                
                // 传统的势场公式 (1/d - 1/d0)，但 Gain 是动态的
                // 斥力方向：障碍物在左，往右(-1)；在右，往左(+1) -> 注意这里符号要对
                // relative_ang > 0 说明障碍物在左边，我们需要向右转(负角速度)
                double direction = (relative_ang > 0) ? -1.0 : 1.0;
                
                double repulsion_w = repulsion_gain * (1.0/std::max(min_dist, 0.1) - 1.0/safe_distance) * direction;
                
                // 叠加斥力
                w_final += repulsion_w;
                
                // 减速机制 (也与模糊因子挂钩)
                // 危险程度越高，减速越狠
                double risk_factor = computeFuzzyAPFGain(min_dist, relative_ang) / 4.0; // 归一化大概
                v_final *= std::max(0.0, 1.0 - risk_factor);
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
 * @brief Execute Fuzzy Adaptive Sliding Mode Control (Fuzzy-SMC)
 */
Eigen::Vector2d PIDController::_modelFreePIDControl(Eigen::Vector3d s,
                                                    Eigen::Vector3d s_d,
                                                    Eigen::Vector2d u_r) {
  Eigen::Vector2d u;

  // 1. 计算误差
  double dx = s_d[0] - s[0];
  double dy = s_d[1] - s[1];
  double path_yaw = std::atan2(dy, dx);
  double e_theta = normalizeAngle(path_yaw - s[2]);

  // 2. >>> 创新点调用：模糊自适应 SMC 参数 <<<
  // 不再是 const double k_approach = 2.0;
  double k_approach = computeFuzzySMCGain(e_theta);
  
  // 切换增益也可以稍微联动一下，通常设为趋近律的一半或固定小值
  double k_switch   = k_approach * 0.5;   
  const double boundary    = 0.15; 

  // 3. 计算 Sat(S)
  double S = e_theta;
  double sat_S = 0.0;
  
  if (S > boundary) sat_S = 1.0;
  else if (S < -boundary) sat_S = -1.0;
  else sat_S = S / boundary;

  // 4. SMC 角速度控制律
  double w_cmd = k_approach * S + k_switch * sat_S;

  // 5. 线速度控制律 (结合投影法)
  double v_cmd = config_.max_linear_velocity() * std::cos(e_theta);

  // 到达终点附近的简单处理
  double dist_to_goal = std::hypot(dx, dy);
  if (dist_to_goal < 0.1) {
    v_cmd = 0.0;
    w_cmd = 0.0;
  }
  
  // 6. 安全限幅
  if (v_cmd < 0.0) v_cmd = 0.0; 
  if (std::fabs(v_cmd) > config_.max_linear_velocity()) {
    v_cmd = std::copysign(config_.max_linear_velocity(), v_cmd);
  }
  if (std::fabs(w_cmd) > config_.max_angular_velocity()) {
    w_cmd = std::copysign(config_.max_angular_velocity(), w_cmd);
  } 
  else if (std::fabs(w_cmd) < config_.min_angular_velocity() && std::fabs(w_cmd) > 0.01) {
    w_cmd = std::copysign(config_.min_angular_velocity(), w_cmd);
  }

  u[0] = v_cmd;
  u[1] = w_cmd;

  return u;
}

Eigen::Vector2d PIDController::_modelBasedPIDControl(Eigen::Vector3d s,
                                                     Eigen::Vector3d s_d,
                                                     Eigen::Vector2d u_r) {
  // 保持原有代码不变...
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