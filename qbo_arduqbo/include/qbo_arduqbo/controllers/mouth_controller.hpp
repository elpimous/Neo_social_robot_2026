#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <qbo_msgs/msg/mouth.hpp>
#include <qbo_msgs/srv/test_leds.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

class MouthController : public rclcpp::Node {
public:
  MouthController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Souscription au topic bouche
  rclcpp::Subscription<qbo_msgs::msg::Mouth>::SharedPtr mouth_sub_;
  rclcpp::Service<qbo_msgs::srv::TestLeds>::SharedPtr test_leds_srv_;
  std::shared_ptr<QboDuinoDriver> driver_;

  // Paramètres
  double rate_;
  std::string topic_;

  // Callback ROS
  void setMouth(const qbo_msgs::msg::Mouth::SharedPtr msg);
  void testMouthLedsCallback(const std::shared_ptr<qbo_msgs::srv::TestLeds::Request>,
                           std::shared_ptr<qbo_msgs::srv::TestLeds::Response>);

  // Diagnostic updater
  diagnostic_updater::Updater updater_;
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
