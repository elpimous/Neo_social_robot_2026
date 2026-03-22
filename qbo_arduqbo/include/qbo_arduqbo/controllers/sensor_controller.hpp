// sensor_controller.hpp (ROS2 / Humble)
#pragma once

#include <memory>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <rclcpp/time.hpp>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "qbo_arduqbo/drivers/qboduino_driver.h"

// NOTE: pas de namespace (aligné avec ImuController)

struct DistanceSensor
{
  std::string name;            // ex: front_left_srf10
  std::string type;            // srf10, VL53L1X, gp2d12, gp2d120, GP2Y0A21YK
  std::string frame_id;        // TF frame
  std::string topic;           // topic de sortie
  uint8_t address = 0;         // I2C ou index ADC
  uint8_t group = 0;           // 1=front, 0=back (autoupdate QBoard)
  bool publish_if_obstacle = false;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub;
  sensor_msgs::msg::PointCloud cloud;
  rclcpp::Time last_seen;
  bool alive{false};
};

class SensorController : public rclcpp::Node
{
public:
  explicit SensorController(std::shared_ptr<QboDuinoDriver> driver,
                            const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // lifecycle
  void loadParameters();
  void setupPublishers();
  void configureBoard();
  void timerCallback();
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & status);

  // helpers
  void createDistanceSensor(const std::string & prefix, const std::string & name, uint8_t group);
  void publishDistance(DistanceSensor & s, unsigned int raw_value, const rclcpp::Time & stamp);

  // state
  std::shared_ptr<QboDuinoDriver> driver_;
  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_;

  // params
  double rate_ = 15.0;                   // Hz
  std::string base_topic_ = "srf10_state"; // prefix topics

  // sensors
  std::map<uint8_t, DistanceSensor> srf10_sensors_;     // par adresse I2C
  std::map<uint8_t, DistanceSensor> adc_sensors_;       // par canal ADC
  std::vector<uint8_t> adc_addresses_;
  std::map<uint8_t, uint8_t> autoupdate_groups_;        // addr -> groupe (front/back)

  // status flags
  bool board_configured_ = false;

  // 🆕 health
  rclcpp::Time last_seen;
  bool alive{false};
};
