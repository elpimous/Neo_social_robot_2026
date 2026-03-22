#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <set>

#include <memory>
#include <string>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <qbo_msgs/msg/lcd.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"


class LcdController : public rclcpp::Node {
public:
    LcdController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Souscriptions
  rclcpp::Subscription<qbo_msgs::msg::LCD>::SharedPtr lcd_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;

  // Timer pour mise à jour régulière
  rclcpp::TimerBase::SharedPtr display_timer_;

  std::shared_ptr<QboDuinoDriver> driver_;
  // rclcpp::TimerBase::SharedPtr lcd_timer_;

  // Callback ROS
  void setLCD(const qbo_msgs::msg::LCD::SharedPtr msg);
  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status);
  void updateLCD();

  // Données extraites des diagnostics
  std::string hostname_ = "";
  std::string ip_address_ = "";
  std::string cpu_temp_ = "--";
  std::string fan_pct_ = "--";
  std::string vdd_in_ = "--";
  std::string soc_ = "--";
  std::string battery_voltage_ = "--";
  std::string est_runtime_ = "--";

  std::string temp_line_override_;
  rclcpp::TimerBase::SharedPtr lcd_reset_timer_;
  bool line_locked_ = false;

  // Mise à jour des lignes du LCD
  std::array<std::string, 4> lcd_lines_;

  // Pour mise à jour LCD par ligne
  std::array<std::string, 4> display_lines_;
  int current_line_ = 0;

  // Diagnostic updater (non utilisé ici, mais prêt si besoin)
  diagnostic_updater::Updater updater_;

  // Etat interne
  double rate_;
  std::string topic_ = "cmd_lcd";
  bool show_hostname_ = true;  // alterne toutes les 5 sec
  bool i2c_status_checked_ = false;
  bool has_lcd_ = false;
};
