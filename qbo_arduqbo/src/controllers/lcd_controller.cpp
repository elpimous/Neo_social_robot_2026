// lcd_controller_ros2.cpp
#include "qbo_arduqbo/controllers/lcd_controller.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

LcdController::LcdController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("lcd_ctrl", "qbo_arduqbo", options),
  driver_(driver),
  updater_(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface(),
    1.0)
{
    // Lecture des paramètres
    get_parameter("topic", topic_);
    get_parameter("rate", rate_);

    uint8_t i2c_state = 0;
    if (driver_->getI2cDevicesState(i2c_state) >= 0) {
        has_lcd_ = i2c_state & 0x08;
        i2c_status_checked_ = true;
        RCLCPP_DEBUG(this->get_logger(), "🧭 I2C Devices State: 0x%02X", i2c_state);
        RCLCPP_DEBUG(this->get_logger(), "    LCD detected: %s", has_lcd_ ? "yes" : "no");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to query LCD device state via I2C at startup.");
    }

    lcd_sub_ = this->create_subscription<qbo_msgs::msg::LCD>(
        topic_, 1, std::bind(&LcdController::setLCD, this, std::placeholders::_1));

    diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10, std::bind(&LcdController::diagCallback, this, std::placeholders::_1));

    updater_.setHardwareID("LCD");
    updater_.add("LCD Status", this, &LcdController::diagnosticCallback);

    display_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(5.0 / rate_),
        std::bind(&LcdController::updateLCD, this));

    RCLCPP_INFO(this->get_logger(), "✅ LCDController initialized with:\n"
                                "       - Rate: %.2f Hz\n"
                                "       - Command topic: %s",
            rate_, topic_.c_str());

    display_lines_[0] = "Hostname: ???";
    // display_lines_[1] = "IP: ???";
    // display_lines_[2] = "Batt: ???";
    // display_lines_[3] = "Temp: ???";

    // Envoie un message initial
    driver_->setLCD("Qbo Ready              ");
}

void LcdController::setLCD(const qbo_msgs::msg::LCD::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "LCD command arrived: %s", msg->msg.c_str());
    std::string content = msg->msg.substr(0, 20);
    content += std::string(20 - content.size(), ' ');

    // Sauvegarder la ligne actuelle
    if (!line_locked_) {
        temp_line_override_ = content;
        line_locked_ = true;
    }

    // Redémarrer le timer pour restaurer après 30s
    if (lcd_reset_timer_) {
        lcd_reset_timer_->cancel();
    }

    lcd_reset_timer_ = this->create_wall_timer(
        std::chrono::seconds(30),
        [this]() {
            line_locked_ = false;
            temp_line_override_.clear();
            lcd_reset_timer_->cancel();  // stop auto
        });
}

void LcdController::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    for (const auto &status : msg->status)
    {
        if (status.name.find("Network") != std::string::npos) {
            for (const auto &v : status.values) {
                if (v.key == "Hostname") hostname_ = "Host: " + v.value;
                if (v.key == "IP Address") ip_address_ = "IP: " + v.value;
            }
        } else if (status.name.find("Battery Status") != std::string::npos) {
            std::string voltage, runtime, charge_mode;
            bool charging = false;

            for (const auto &v : status.values) {
                if (v.key == "Voltage (V)") voltage = v.value;
                if (v.key == "Estimated Runtime (min)") runtime = v.value;
                if (v.key == "Charge Mode Description") charge_mode = v.value;
                if (v.key == "External Power" && v.value == "Yes") charging = true;
            }

            if (charging && !charge_mode.empty()) {
                display_lines_[1] = charge_mode;
            } else if (!voltage.empty() && !runtime.empty()) {
                try {
                    int runtime_min = static_cast<int>(std::stod(runtime));
                    int h = runtime_min / 60;
                    int m = runtime_min % 60;
                    std::ostringstream oss;
                    oss << "Batt: " << voltage << "V "
                        << h << "h" << std::setfill('0') << std::setw(2) << m;
                    display_lines_[1] = oss.str();
                } catch (...) {
                    display_lines_[1] = "Batt: " + voltage + "V";
                }
            } else {
                display_lines_[1] = "Batt: " + voltage + "V";
            }
        } else if (status.name.find("Temp") != std::string::npos) {
            for (const auto &v : status.values) {
                if (v.key == "CPU °C") display_lines_[3] = "Temp: " + v.value + "C";
            }
        }
    }
}


void LcdController::updateLCD()
{
    auto padOrTrim = [](const std::string& input, size_t width = 20) -> std::string {
        if (input.size() >= width)
            return input.substr(0, width);
        return input + std::string(width - input.size(), ' ');
    };

    std::string line0 = show_hostname_ ? hostname_ : ip_address_;
    show_hostname_ = !show_hostname_;  // alterner à chaque appel (5s)

    std::string lines[4];
    lines[0] = padOrTrim(line0);
    if (line_locked_ && !temp_line_override_.empty()) {
        lines[1] = padOrTrim(temp_line_override_);
    } else {
        lines[1] = padOrTrim(display_lines_[1]);
    }
    lines[2] = padOrTrim(display_lines_[2]);
    lines[3] = padOrTrim(display_lines_[3]);

    driver_->setLCD(lines[0]);
    // Si ton LCD supporte des commandes par ligne :
    driver_->setLCD("1" + lines[1]);
    // driver_->setLCD("2" + lines[2]);
    // driver_->setLCD("3" + lines[3]);
}

void LcdController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status) {
    status.summary(
        (has_lcd_) ? diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        (i2c_status_checked_ ? "LCD Operational" : "LCD I2C check failed at startup"));

    // 🟢 État dynamique
    status.add("LCD Present", has_lcd_ ? "yes" : "no");

    // 🟣 Infos techniques statiques
    status.add("LCD Model", "C2042A");
    status.add("I2C Address", "0x63");

    // 🔴 Message explicite si erreur
    if (!has_lcd_) {
        std::string msg = std::string(!has_lcd_ ? "LCD missing" : "");
        status.message = msg;
    }

}

