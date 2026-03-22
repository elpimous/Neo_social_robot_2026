#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/drivers/i2c_bus_driver.hpp"
#include "qbo_arduqbo/drivers/qboduino_driver.h"
#include "qbo_arduqbo/controllers/battery_controller.hpp"
#include "qbo_arduqbo/controllers/base_controller.hpp"
#include "qbo_arduqbo/controllers/imu_controller.hpp"
#include "qbo_arduqbo/controllers/lcd_controller.hpp"
#include "qbo_arduqbo/controllers/nose_controller.hpp"
#include "qbo_arduqbo/controllers/mouth_controller.hpp"
#include "qbo_arduqbo/controllers/audio_controller.hpp"
#include "qbo_arduqbo/controllers/sensor_controller.hpp"

class QboArduqboManager {
public:
    explicit QboArduqboManager(std::shared_ptr<rclcpp::Node> node,
                                const rclcpp::NodeOptions& options,
                                const std::string &port1,
                                const std::string &port2);
    void setup();
    void run();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<I2CBusDriver> i2c_driver_;
    std::shared_ptr<QboDuinoDriver> arduino_driver_;
    std::vector<std::shared_ptr<rclcpp::Node>> controllers_;  // Pour garder les instances
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::NodeOptions node_options_;
    std::string port1_;
    std::string port2_;

    // Flags
    bool enable_qboard1_ = false;
    bool enable_qboard2_ = false;
    bool enable_battery_ = false;
    bool enable_imu_base_ = false;
    bool enable_base_ = false;
    bool enable_lcd_ = false;
    bool enable_nose_ = false;
    bool enable_mouth_ = false;
    bool enable_audio_ = false;
    bool enable_sensors_ = false;

    int qboard1_version_ = -1;
    int qboard2_version_ = -1;
    int baud1_ = 115200;
    int baud2_ = 115200;
    double timeout1_ = 0.5;
    double timeout2_ = 0.5;
    uint8_t id = 0;
    int board_id = -1, version = -1;

    // Diagnostics
    std::unique_ptr<diagnostic_updater::Updater> updater_;

    // ➕ Fonctions utilitaires internes
    void logControllerStatus(const std::string &name, bool enabled, bool loaded);

};
