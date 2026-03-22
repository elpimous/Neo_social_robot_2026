// imu_controller_ros2.cpp
#include "qbo_arduqbo/controllers/imu_controller.hpp"

static constexpr double GYRO_MEASUREMENT_SCALE = 250.0 * M_PI / (180.0 * 32768.0);
static constexpr double ACC_MEASUREMENT_SCALE = 9.81 / 16384.0;

ImuController::ImuController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("imu_ctrl", "qbo_arduqbo", options),
  updater_(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface(),
    1.0),
    driver_(driver),
    is_calibrated_(true),
    is_calibrating_(false)
{
    uint8_t i2c_state = 0;

    get_parameter("topic", topic_);
    get_parameter("rate", rate_);

    if (driver_->getI2cDevicesState(i2c_state) >= 0) {
        has_gyro_ = i2c_state & 0x02;
        has_accel_ = i2c_state & 0x04;
        i2c_status_checked_ = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unable to query IMU device state via I2C at startup.");
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_ + "/data", 10);
    imu_calibrated_pub_ = this->create_publisher<std_msgs::msg::Bool>(topic_ + "/is_calibrated", 10);

    calibrate_service_ = this->create_service<qbo_msgs::srv::CalibrateIMU>(
        topic_ + "/calibrate",
        std::bind(&ImuController::calibrateService, this, std::placeholders::_1, std::placeholders::_2)
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
        std::bind(&ImuController::timerCallback, this)
    );

    updater_.setHardwareID("Qboard_4");
    updater_.add("IMU Status", this, &ImuController::diagnosticCallback);

    imu_msg_.header.frame_id = "base_link";
    imu_msg_.orientation_covariance = { -1.0, 0, 0, 0, -1.0, 0, 0, 0, -1.0 };
    imu_msg_.angular_velocity_covariance = { 0.008, 0, 0, 0, 0.008, 0, 0, 0, 0.008 };
    imu_msg_.linear_acceleration_covariance = { 0.002, 0, 0, 0, 0.002, 0, 0, 0, 0.002 };

    imu_calibrated_.data = true;
    last_calibration_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "✅ ImuController initialized — Rate: %.2f Hz, Topic: %s",
                rate_, topic_.c_str());
}

void ImuController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status)
{
    bool hardware_ok = has_gyro_ && has_accel_;

    if (!hardware_ok) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU hardware incomplete");
    } else if (is_calibrating_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU calibration in progress...");
    } else if (!is_calibrated_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU not calibrated");
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "IMU operational");
    }

    status.add("Gyroscope Present", has_gyro_ ? "yes" : "no");
    status.add("Accelerometer Present", has_accel_ ? "yes" : "no");
    status.add("Accelerometer Model", "LIS35DE");
    status.add("I2C Address Accelerometer", "0x1C");
    status.add("Gyroscope Model", "L3G400D");
    status.add("I2C Address Gyroscope", "0x69");
    status.add("IMU calibrated", is_calibrated_ ? "yes" : "no");
    status.add("Calibration in progress", is_calibrating_ ? "yes" : "no");

    if (is_calibrated_) {
        rclcpp::Duration since = this->now() - last_calibration_time_;
        int s = static_cast<int>(since.seconds());
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%02dh:%02dm:%02ds ago", s / 3600, (s % 3600) / 60, s % 60);
        status.add("Last calibration", std::string(buffer));
    } else {
        status.add("Last calibration", "N/A");
    }
}

void ImuController::timerCallback()
{
    if (is_calibrating_) return;

    int16_t gx, gy, gz, ax, ay, az;
    int code = driver_->getIMU(gx, gy, gz, ax, ay, az);

    if (code < 0) {
        RCLCPP_ERROR(get_logger(), "Unable to get IMU data from the base controller board");
        has_gyro_ = false;
        has_accel_= false;
        return;
    }

    imu_msg_.angular_velocity.x = static_cast<float>(gx) * GYRO_MEASUREMENT_SCALE;
    imu_msg_.angular_velocity.y = static_cast<float>(gy) * GYRO_MEASUREMENT_SCALE;
    imu_msg_.angular_velocity.z = static_cast<float>(gz) * GYRO_MEASUREMENT_SCALE;
    imu_msg_.linear_acceleration.x = static_cast<float>(ax) * ACC_MEASUREMENT_SCALE;
    imu_msg_.linear_acceleration.y = static_cast<float>(ay) * ACC_MEASUREMENT_SCALE;
    imu_msg_.linear_acceleration.z = static_cast<float>(az) * ACC_MEASUREMENT_SCALE;
    imu_msg_.header.stamp = this->now();

    imu_pub_->publish(imu_msg_);
    imu_calibrated_pub_->publish(imu_calibrated_);

    has_gyro_ = true;
    has_accel_= true;
}

bool ImuController::calibrateService(
    const std::shared_ptr<qbo_msgs::srv::CalibrateIMU::Request>,
    std::shared_ptr<qbo_msgs::srv::CalibrateIMU::Response> res)
{
    is_calibrating_ = true;
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "📡 Starting IMU calibration (non-blocking firmware)...");

    // ✅ Polling loop avec la bonne sémantique du nouveau firmware
    //
    // Protocole calibrateRequest() côté Arduino :
    //   Premier appel  → démarre la machine d'état, répond 0xFF
    //   Appels suivants → 0xFF tant que CALIB_RUNNING dans spinOnce()
    //   Quand terminé  → répond 1 (succès) ou 0 (échec), reset état
    //
    // Cette boucle est donc correcte : chaque appel calibrateIMU() est un poll légitime.
    // Le firmware ne redémarre pas la calibration si déjà en cours.
    // Timeout : 30s (500 samples × 5ms = ~2.5s de collecte + marge confortable)

    uint8_t result = 0;
    const int POLL_INTERVAL_MS = 500;
    const int TIMEOUT_MS       = 30000;
    int elapsed_ms = 0;

    // Premier appel : démarre la calibration
    int code = driver_->calibrateIMU(result);
    if (code < 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to start IMU calibration (code: %d)", code);
        res->success = false;
        res->message = "Serial error at calibration start.";
        is_calibrated_ = false;
        imu_calibrated_.data = false;
        is_calibrating_ = false;
        timer_->reset();
        return true;
    }

    // Polling jusqu'au résultat final
    while (result == 0xFF && elapsed_ms < TIMEOUT_MS) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⏳ Calibration in progress... (%d/%dms)", elapsed_ms, TIMEOUT_MS);
        rclcpp::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
        elapsed_ms += POLL_INTERVAL_MS;
        code = driver_->calibrateIMU(result);
        if (code < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Serial error during poll (code: %d)", code);
            break;
        }
    }

    if (code < 0 || result == 0) {
        res->success = false;
        res->message = (elapsed_ms >= TIMEOUT_MS) ? "IMU calibration timeout." : "IMU calibration failed.";
        RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
        is_calibrated_ = false;
        imu_calibrated_.data = false;
    } else {
        res->success = true;
        res->message = "IMU calibration successful.";
        RCLCPP_INFO(this->get_logger(), "✅ IMU calibration successful (%dms)", elapsed_ms);
        is_calibrated_ = true;
        imu_calibrated_.data = true;
        last_calibration_time_ = this->now();
    }

    // reset() systématique — succès ET échec
    is_calibrating_ = false;
    timer_->reset();

    return true;
}
