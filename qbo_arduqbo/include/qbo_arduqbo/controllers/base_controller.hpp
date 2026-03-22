#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <set>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include "qbo_msgs/srv/set_odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "qbo_arduqbo/drivers/qboduino_driver.h"

class BaseController : public rclcpp::Node {
public:
  BaseController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  void publishStaticTF();
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status);

  bool resetStallService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                  std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool stopBaseService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool setOdometryService(const std::shared_ptr<qbo_msgs::srv::SetOdometry::Request>,
                   std::shared_ptr<qbo_msgs::srv::SetOdometry::Response>);

  double estimateMotorsPowerFromOdometry(double linear_speed, double angular_speed);

  // Driver série vers la base Q.bo
  std::shared_ptr<QboDuinoDriver> driver_;

  // Paramètres utilisateur
  std::string cmd_topic_, odom_topic_;
  double rate_;
  bool broadcast_tf_, base_stop_;

  // État interne du robot
  float v_linear_, v_angular_, x_, y_, th_;
  bool v_dirty_;
  double last_estimated_motor_power_ = 0.0;
  bool last_odometry_ok_ = false;

  // Interfaces ROS 2
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_stall_srv_, stop_base_srv_;
  rclcpp::Service<qbo_msgs::srv::SetOdometry>::SharedPtr set_odometry_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  bool static_tf_sent_ = false;

  rclcpp::Time last_time_;
};
