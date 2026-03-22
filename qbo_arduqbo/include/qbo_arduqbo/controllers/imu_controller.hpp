#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <qbo_msgs/srv/calibrate_imu.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"

class ImuController : public rclcpp::Node {
public:
    ImuController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void timerCallback();
    bool calibrateService(
        const std::shared_ptr<qbo_msgs::srv::CalibrateIMU::Request>,
        std::shared_ptr<qbo_msgs::srv::CalibrateIMU::Response>
    );
    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status);

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_calibrated_pub_;
    rclcpp::Service<qbo_msgs::srv::CalibrateIMU>::SharedPtr calibrate_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    diagnostic_updater::Updater updater_;
    rclcpp::Time last_calibration_time_;

    // Etat interne
    std::shared_ptr<QboDuinoDriver> driver_;
    sensor_msgs::msg::Imu imu_msg_;
    std_msgs::msg::Bool imu_calibrated_;
    double rate_ = 10;
    std::string topic_ = "imu_state";
    bool is_calibrated_;
    bool is_calibrating_;
    double last_calibrated_;
    bool has_gyro_ = false;
    bool has_accel_ = false;
    bool i2c_status_checked_ = false;
};
