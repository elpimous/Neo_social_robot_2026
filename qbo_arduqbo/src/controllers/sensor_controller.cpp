// sensor_controller.cpp (ROS2 / Humble)
#include "qbo_arduqbo/controllers/sensor_controller.hpp"

#include <cmath>
#include <algorithm>

// NOTE: pas de namespace (aligné avec ImuController)

static inline double gp2d12_to_m(unsigned int raw)
{
  if (raw < 3) return -1.0;
  return (6787.0 / (static_cast<double>(raw) - 3.0)) - 4.0;
}

static inline double gp2d120_to_m(unsigned int raw)
{
  return (2914.0 / (static_cast<double>(raw) + 5.0)) - 1.0;
}

static inline double gp2y0a21yk_to_m(unsigned int raw)
{
  return (12343.85 * std::pow(static_cast<double>(raw), -1.15)) / 100.0;
}

SensorController::SensorController(std::shared_ptr<QboDuinoDriver> driver,
                                   const rclcpp::NodeOptions & options)
: rclcpp::Node("sens_ctrl", "qbo_arduqbo", options),
  driver_(std::move(driver)),
  updater_(this->get_node_base_interface(),
           this->get_node_clock_interface(),
           this->get_node_logging_interface(),
           this->get_node_parameters_interface(),
           this->get_node_timers_interface(),
           this->get_node_topics_interface(),
           1.0)
{
  // Declare top-level params with defaults (ROS2 style)
//   this->declare_parameter<double>("rate", rate_);
//   this->declare_parameter<std::string>("topic", base_topic_);

  // Load + setup
  loadParameters();
  setupPublishers();
  configureBoard();

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
      std::bind(&SensorController::timerCallback, this));

  // 🔍 Diagnostic setup
  updater_.setHardwareID("Qboard_1");
  updater_.add("Sensors Status", this, &SensorController::diagnosticCallback);

  RCLCPP_INFO(this->get_logger(),
              "✅ SensorController initialized: rate=%.2f Hz, base_topic=%s, SRF10=%zu, ADC=%zu",
              rate_, base_topic_.c_str(), srf10_sensors_.size(), adc_sensors_.size());
}

void SensorController::loadParameters()
{
  // Fallbacks par défaut (comme IMU)
  if (!this->get_parameter("rate", rate_)) {
    rate_ = 15.0;
  }
  if (!this->get_parameter("topic", base_topic_)) {
    base_topic_ = "srf10_state";
  }

  // Découverte des capteurs
  auto listed = this->list_parameters({"sensors"}, 10);

  std::set<std::pair<std::string, std::string>> sensor_entries;
  for (const auto & full : listed.names) {
    const std::string prefix = "sensors.";
    if (full.rfind(prefix, 0) != 0) continue;
    auto rest = full.substr(prefix.size());
    auto first_dot = rest.find('.');
    if (first_dot == std::string::npos) continue;
    std::string group = rest.substr(0, first_dot);
    auto next = rest.substr(first_dot + 1);
    auto second_dot = next.find('.');
    if (second_dot == std::string::npos) continue;
    std::string name = next.substr(0, second_dot);
    sensor_entries.insert({group, name});
  }

  for (const auto & pair : sensor_entries) {
    const auto & group = pair.first;
    const auto & name  = pair.second;
    uint8_t group_id = (group == "front") ? 1 : 0;
    const std::string base = "sensors." + group + "." + name;

    createDistanceSensor(base, name, group_id);
  }
}


void SensorController::createDistanceSensor(const std::string & prefix,
                                            const std::string & name,
                                            uint8_t group)
{
  std::string type;      if (!this->get_parameter(prefix + ".type", type)) type = "srf10";
  int address_i = 0;     (void)this->get_parameter(prefix + ".address", address_i);
  std::string frame;     (void)this->get_parameter(prefix + ".frame_id", frame);
  std::string topic;     (void)this->get_parameter(prefix + ".topic", topic);
  bool pub_if_obs = false; (void)this->get_parameter(prefix + ".publish_if_obstacle", pub_if_obs);

  if (address_i <= 0) {
    RCLCPP_WARN(this->get_logger(), "You need to set a valid address for sensor '%s' (prefix=%s)",
                name.c_str(), prefix.c_str());
    return;
  }
  if (topic.empty()) topic = base_topic_ + "/" + name;

  DistanceSensor s;
  s.name = name;
  s.type = type;
  s.frame_id = frame;
  s.topic = topic;
  s.address = static_cast<uint8_t>(address_i);
  s.group = group;
  s.publish_if_obstacle = pub_if_obs;
  s.cloud.points.resize(1);
  s.cloud.header.frame_id = frame;

  if (type == "srf10" || type == "VL53L1X") {
    srf10_sensors_.emplace(s.address, s);
    autoupdate_groups_[s.address] = group;
  } else if (type == "gp2d12" || type == "gp2d120" || type == "GP2Y0A21YK") {
    adc_sensors_.emplace(s.address, s);
    adc_addresses_.push_back(s.address);
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown sensor type '%s' for '%s' — ignored", type.c_str(), name.c_str());
  }
}


void SensorController::setupPublishers()
{
  for (auto & kv : srf10_sensors_) {
    auto & s = kv.second;
    s.pub = this->create_publisher<sensor_msgs::msg::PointCloud>(s.topic, 10);
  }
  for (auto & kv : adc_sensors_) {
    auto & s = kv.second;
    s.pub = this->create_publisher<sensor_msgs::msg::PointCloud>(s.topic, 10);
  }
}

void SensorController::configureBoard()
{
  // configure autoupdate for SRF10 / VL53L1X
  int code = driver_->setAutoupdateSensors(autoupdate_groups_);
  if (code < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Unable to activate all SRF10 sensors on the base control board");
    board_configured_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(),
                "All SRF10 sensors correctly activated on base control board");
    board_configured_ = true;
  }
}

void SensorController::publishDistance(DistanceSensor & s,
                                       unsigned int raw,
                                       const rclcpp::Time & stamp)
{
  double distance_m = 0.0;
  if (s.type == "srf10" || s.type == "VL53L1X") {
    distance_m = static_cast<double>(raw) / 100.0; // cm → m
  } else if (s.type == "gp2d120") {
    distance_m = gp2d120_to_m(raw);
  } else if (s.type == "gp2d12") {
    distance_m = gp2d12_to_m(raw);
  } else if (s.type == "GP2Y0A21YK") {
    distance_m = gp2y0a21yk_to_m(raw);
  }

  s.cloud.points[0].x = static_cast<float>(distance_m);
  s.cloud.points[0].y = 0.0f;
  s.cloud.points[0].z = 0.0f;
  s.cloud.header.stamp = stamp;

  if (!s.publish_if_obstacle || raw > 0) {
    s.pub->publish(s.cloud);
  }
  s.last_seen = stamp;
  s.alive = true;
}

void SensorController::timerCallback()
{
  const auto now = this->now();

  // 1) SRF10 / VL53L1X
  std::map<uint8_t, unsigned short> updated;
  int code = driver_->getDistanceSensors(updated);
  if (code < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Unable to get SRF10 distances from the base control board");
  } else {
    for (auto & kv : srf10_sensors_) {
      const auto addr = kv.first;
      auto & s = kv.second;
      if (updated.count(addr) > 0) {
        publishDistance(s, static_cast<unsigned int>(updated[addr]), now);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No distance for SRF10 sensor 0x%02X (%s)",
                             addr, s.name.c_str());
      }
    }
  }

  // 2) ADC (Sharp IR)
  if (!adc_addresses_.empty()) {
    std::vector<unsigned int> reads;
    code = driver_->getAdcReads(adc_addresses_, reads);
    if (code < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unable to get ADC sensor reads from the base control board");
    } else if (reads.size() != adc_addresses_.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "ADC addresses and returned reads do not match");
    } else {
      for (size_t i = 0; i < adc_addresses_.size(); ++i) {
        auto addr = adc_addresses_[i];
        auto it = adc_sensors_.find(addr);
        if (it == adc_sensors_.end()) continue;
        publishDistance(it->second, reads[i], now);
      }
    }
  }

  // updater_.force_update();
}

void SensorController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  if (srf10_sensors_.empty() && adc_sensors_.empty()) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No sensors configured");
    return;
  }

  const auto now = this->now();
  const double timeout = 2.0;  // secondes sans données → WARN

  int missing = 0;

  for (const auto & kv : srf10_sensors_) {
    const auto & s = kv.second;
    if (!s.alive || (now - s.last_seen).seconds() > timeout) {
      missing++;
      status.add("Missing SRF10", s.name);
    }
  }

  for (const auto & kv : adc_sensors_) {
    const auto & s = kv.second;
    if (!s.alive || (now - s.last_seen).seconds() > timeout) {
      missing++;
      status.add("Missing ADC", s.name);
    }
  }

  if (missing > 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "One or more sensors not responding");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "Sensors operational");
  }

  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Sensors operational");
  status.add("SRF10 count", static_cast<int>(srf10_sensors_.size()));
  status.add("ADC count", static_cast<int>(adc_sensors_.size()));
  status.add("Board configured", board_configured_ ? "yes" : "no");
}
