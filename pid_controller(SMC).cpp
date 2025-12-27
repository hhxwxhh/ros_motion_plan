/**
 * *********************************************************
 *
 * @file: pid_controller.cpp
 * @brief: Modified to Sliding Mode Controller (SMC) for Robust Trajectory Tracking
 * @author: Wang Xuhui
 * @date: 2025-12-25
 * @version: 2.1 (SMC Version - Fixed Compilation Error)
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "common/geometry/point.h"
#include "controller/pid_controller.h"

// 引入必要的数学库
#include <cmath>
#include <algorithm>

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

    // PID的历史变量（虽然SMC不用积分，但保留变量定义防止报错）
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    // 修改日志输出，表明当前是 SMC 模式
    R_INFO << "Sliding Mode Controller (SMC) initialized for Robust Tracking!";
  } else {
    R_WARN << "Controller has already been initialized.";
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
    R_ERROR << "Controller has not been initialized";
    return false;
  }

  if (goal_reached_) {
    R_INFO << "GOAL Reached!";
    return true;
  }
  return false;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the
 * velocity commands
 */
bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "Controller has not been initialized";
    return false;
  }

  // odometry observation
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in map
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, config_.map_frame(), robot_pose_odom, robot_pose_map);

  // prune the global plan
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_odom);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  
  // 动态调整预瞄距离 L
  double L = clamp(std::fabs(vt) * pid_config_.lookahead_time(),
                   pid_config_.min_lookahead_dist(), pid_config_.max_lookahead_dist());

  // get the particular point on the path at the lookahead distance
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]

  // === 这里的逻辑保持不变：如果到了终点附近，就只调整角度 ===
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    double e_theta = normalizeAngle(goal_theta_ - theta);

    if (!shouldRotateToPath(std::fabs(e_theta))) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    else {
      cmd_vel.linear.x = 0.0;
      // 这里也可以用 SMC 的一部分逻辑：
      double k_rotate = 1.0; 
      double w = k_rotate * e_theta + 0.5 * (e_theta > 0 ? 1.0 : -1.0); // 简单的带sgn的旋转
      cmd_vel.angular.z = clamp(w, -config_.max_angular_velocity(), config_.max_angular_velocity());
    }
  }
  // === 正常的路径跟踪模式 ===
  else {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y,
                      theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.x(), lookahead_pt.y(),
                        lookahead_pt.theta());  // desired state
    Eigen::Vector2d u_r(vt, wt);                // refered input
    
    // 这里调用我们魔改后的函数 _modelFreePIDControl (实际执行的是SMC)
    Eigen::Vector2d u = pid_config_.model_based_mode() ?
                            _modelBasedPIDControl(s, s_d, u_r) :
                            _modelFreePIDControl(s, s_d, u_r);

    cmd_vel.linear.x = linearRegularization(vt, u[0]);
    cmd_vel.angular.z = angularRegularization(wt, u[1]);
  }

  // visualization
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
 * @brief Execute Sliding Mode Control (Formerly Model-Free PID)
 * @notice 虽然函数名还叫 PID，但内部逻辑已经被替换为 SMC 以完成创新点
 * * @param s   current state [x, y, theta]
 * @param s_d desired state (lookahead point) [x_d, y_d, theta_d]
 * @param u_r refered input [v_ref, w_ref]
 * @return u  control vector [v_cmd, w_cmd]
 */
Eigen::Vector2d PIDController::_modelFreePIDControl(Eigen::Vector3d s,
                                                    Eigen::Vector3d s_d,
                                                    Eigen::Vector2d u_r) {
  // ========================================================================
  // === INNOVATION START: SLIDING MODE CONTROLLER (SMC) Implementation ===
  // ========================================================================
  
  Eigen::Vector2d u;

  // 1. 计算误差
  double dx = s_d[0] - s[0];
  double dy = s_d[1] - s[1];
  
  // 计算指向预瞄点的期望航向角 (Desired Heading)
  double path_yaw = std::atan2(dy, dx);
  
  // 计算航向误差 (Heading Error)
  // normalizeAngle 是项目自带的函数，确保误差在 [-PI, PI] 之间
  double e_theta = normalizeAngle(path_yaw - s[2]);

  // 2. SMC 参数设置 (可以在这里调整，或者写到 yaml 配置文件中)
  // k_approach: 指数趋近项系数，决定收敛速度
  const double k_approach = 2.0; 
  // k_switch: 切换增益 (Switching Gain)，这是抗扰的核心参数！
  // 即使有风（干扰），这个项会强制修正
  const double k_switch = 1.0;   
  // boundary: 饱和层边界厚度，用于消除抖振 (Chattering)
  const double boundary = 0.15;  

  // 3. 计算滑模控制律 (Angular Velocity)
  // 滑模面 S 定义为航向误差 e_theta
  double S = e_theta;
  
  // 计算符号函数 sgn(S) 的平滑版本 (Sat函数)
  double sat_S = 0.0;
  if (S > boundary) sat_S = 1.0;
  else if (S < -boundary) sat_S = -1.0;
  else sat_S = S / boundary; // 在边界内线性过渡

  // SMC 控制公式: w = k * e + rho * sgn(e)
  double w_cmd = k_approach * S + k_switch * sat_S;

  // 4. 计算线速度控制律 (Linear Velocity)
  // 如果航向误差很大，减速；如果对准了，全速前进
  double dist_to_goal = std::hypot(dx, dy);
  
  // ==================== FIX HERE ====================
  // 使用 config_.max_linear_velocity() 而不是 pid_config_.max_linear_velocity()
  double v_cmd = config_.max_linear_velocity() * std::cos(e_theta);
  // ==================================================

  // 到达终点判定（简单的距离刹车）
  if (dist_to_goal < 0.1) {
    v_cmd = 0.0;
    w_cmd = 0.0;
  }
  
  // 5. 速度限幅 (Safety Saturation)
  if (v_cmd < 0.0) v_cmd = 0.0; // 简单起见，不许倒车
  if (std::fabs(v_cmd) > config_.max_linear_velocity()) {
    v_cmd = std::copysign(config_.max_linear_velocity(), v_cmd);
  }
  
  // 角速度限幅
  if (std::fabs(w_cmd) > config_.max_angular_velocity()) {
    w_cmd = std::copysign(config_.max_angular_velocity(), w_cmd);
  } else if (std::fabs(w_cmd) < config_.min_angular_velocity() && std::fabs(w_cmd) > 0.01) {
     // 保持最小转速以克服静摩擦
    w_cmd = std::copysign(config_.min_angular_velocity(), w_cmd);
  }

  // 6. 输出指令
  u[0] = v_cmd;
  u[1] = w_cmd;

  // 调试信息 (可选，用来在终端看 SMC 是否生效)
  // R_INFO << "SMC: err=" << e_theta << " sat=" << sat_S << " w=" << w_cmd;

  return u;

  // ========================================================================
  // === INNOVATION END ===
  // ========================================================================
}

/**
 * @brief Execute model-based PID control process
 */
Eigen::Vector2d PIDController::_modelBasedPIDControl(Eigen::Vector3d s,
                                                     Eigen::Vector3d s_d,
                                                     Eigen::Vector2d u_r) {
  // Model Based 模式这里暂不修改
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