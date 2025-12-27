/**
 * *********************************************************
 *
 * @file: pid_controller.h
 * @brief: Contains the PID controller local controller class
 * (Modified for Fuzzy Adaptive SMC + APF)
 * @author: Wang Xuhui
 * @date: 2025-12-25
 * @version: 2.0
 *
 * ********************************************************
 */

#ifndef RMP_CONTROLLER_PID_CONTROLLER_H_
#define RMP_CONTROLLER_PID_CONTROLLER_H_

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include "controller/controller.h"
#include "system_config/controller_protos/pid_controller.pb.h"

namespace rmp {
namespace controller {
/**
 * @brief A class implementing a local controller using the PID (Enhanced with Fuzzy Logic)
 */
class PIDController : public nav_core::BaseLocalPlanner, Controller {
public:
  /**
   * @brief Construct a new PIDController object
   */
  PIDController();

  /**
   * @brief Construct a new PIDController object
   */
  PIDController(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the PIDController object
   */
  ~PIDController();

  /**
   * @brief Initialization of the local controller
   */
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Check if the goal pose has been achieved
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, compute the velocity commands
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  /**
   * @brief Execute model-based PID control process
   */
  Eigen::Vector2d _modelBasedPIDControl(Eigen::Vector3d s, Eigen::Vector3d s_d,
                                        Eigen::Vector2d u_r);

  /**
   * @brief Execute model-free PID control process (Now uses Fuzzy SMC)
   */
  Eigen::Vector2d _modelFreePIDControl(Eigen::Vector3d s, Eigen::Vector3d s_d,
                                       Eigen::Vector2d u_r);

  // ==========================================
  // >>> 新增：模糊自适应辅助函数声明 <<<
  // ==========================================
  
  /**
   * @brief 基于模糊规则计算 SMC 的趋近律增益
   * @details 使用 Sigmoid 函数根据航向误差自适应调整增益
   * @param error 航向误差 (Heading Error)
   * @return 自适应增益 k
   */
  double computeFuzzySMCGain(double error);

  /**
   * @brief 基于模糊规则计算 APF 的斥力权重
   * @details 综合考虑障碍物距离(Distance)和相对角度(Relative Angle)
   * @param dist 障碍物距离
   * @param angle 障碍物相对角度
   * @return 自适应斥力增益
   */
  double computeFuzzyAPFGain(double dist, double angle);
  
  // ==========================================
  // >>> 结束新增部分 <<<
  // ==========================================

private:
  pb::controller::PIDController pid_config_;

  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  double e_v_, e_w_;
  double i_v_, i_w_;

  ros::Publisher target_pt_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};
};  // namespace controller
}  // namespace rmp
#endif