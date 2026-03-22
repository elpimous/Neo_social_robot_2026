#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <qbo_msgs/msg/nose.hpp>
#include <qbo_msgs/srv/test_leds.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>



class NoseController : public rclcpp::Node {
public:
  NoseController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Souscription au topic du nez
  rclcpp::Subscription<qbo_msgs::msg::Nose>::SharedPtr nose_sub_;
  rclcpp::Service<qbo_msgs::srv::TestLeds>::SharedPtr test_leds_srv_;

  std::shared_ptr<QboDuinoDriver> driver_;

  // Diagnostic
  diagnostic_updater::Updater updater_;
  uint8_t last_color_{0};

  // Paramètres
  double rate_;
  std::string topic_;

  // Callback ROS
  void setNose(const qbo_msgs::msg::Nose::SharedPtr msg);
  void testNoseLedsCallback(const std::shared_ptr<qbo_msgs::srv::TestLeds::Request>,
                           std::shared_ptr<qbo_msgs::srv::TestLeds::Response>);
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
