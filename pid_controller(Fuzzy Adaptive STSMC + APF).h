/**
 * *********************************************************
 * @file: pid_controller.h
 * @brief: Robust Controller combining Fuzzy STSMC (for tracking) and APF (for avoidance)
 * @author: Wang Xuhui
 * @date: 2025-12-25
 * @version: 3.0 (Fuzzy STSMC + APF)
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
 * @brief A class implementing a local controller using Fuzzy Adaptive STSMC + APF
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
   * @param name        the name to give this instance of the trajectory controller
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Check if the goal pose has been achieved
   * @return true if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute
   * the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  /**
   * @brief Execute model-based PID control process
   * @param s   current state
   * @param s_d desired state
   * @param u_r refered input
   * @return u  control vector
   */
  Eigen::Vector2d _modelBasedPIDControl(Eigen::Vector3d s, Eigen::Vector3d s_d,
                                        Eigen::Vector2d u_r);

  /**
   * @brief Execute model-free control (Implemented as Fuzzy STSMC)
   * @param s   current state
   * @param s_d desired state
   * @param u_r refered input
   * @return u  control vector
   */
  Eigen::Vector2d _modelFreePIDControl(Eigen::Vector3d s, Eigen::Vector3d s_d,
                                       Eigen::Vector2d u_r);

  // ==========================================
  // >>> 新增：模糊自适应辅助函数 <<<
  // ==========================================
  
  /**
   * @brief 基于模糊规则计算 SMC 的扰动等级增益
   * @param error 航向误差
   * @return 自适应增益
   */
  double computeFuzzySMCGain(double error);

  /**
   * @brief 基于模糊规则计算 APF 的斥力权重
   * @param dist 障碍物距离
   * @param angle 障碍物相对角度
   * @return 自适应斥力增益
   */
  double computeFuzzyAPFGain(double dist, double angle);
  
  // ==========================================

private:
  pb::controller::PIDController pid_config_;

  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  double e_v_, e_w_;
  double i_v_, i_w_;

  // >>> 新增：Super-Twisting 积分项 <<<
  double stsmc_integral_v_; 

  ros::Publisher target_pt_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};
};  // namespace controller
}  // namespace rmp
#endif