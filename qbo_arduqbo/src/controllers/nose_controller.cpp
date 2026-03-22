#include "qbo_arduqbo/controllers/nose_controller.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {
bool last_nose_action_ok = true;
}

NoseController::NoseController(
    std::shared_ptr<QboDuinoDriver> driver,
    const rclcpp::NodeOptions & options)
    : Node("nose_ctrl", "qbo_arduqbo", options),
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
    this->get_parameter("topic", topic_);
    this->get_parameter("rate", rate_);

    nose_sub_ = this->create_subscription<qbo_msgs::msg::Nose>(
        topic_, 10,
        std::bind(&NoseController::setNose, this, std::placeholders::_1)
    );

    test_leds_srv_ = this->create_service<qbo_msgs::srv::TestLeds>(
        this->get_name() + std::string("/test_leds"),
        std::bind(&NoseController::testNoseLedsCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 🔍 Diagnostic setup
    updater_.setHardwareID("Qboard_5");
    updater_.add("Nose Status", this, &NoseController::produceDiagnostics);

    RCLCPP_INFO(this->get_logger(), "✅ NoseController initialized with:\n"
                                "       - Rate: %.2f Hz\n"
                                "       - Command topic: %s",
            rate_, topic_.c_str());
}

void NoseController::setNose(const qbo_msgs::msg::Nose::SharedPtr msg)
{
    if (msg->color > 7) {
        RCLCPP_WARN(this->get_logger(), "Nose color %d is invalid. Must be between 0 and 7.", msg->color);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Nose command received: %d", msg->color);

    last_color_ = msg->color;

    int code = driver_->setNose(msg->color);
    if (code < 0) {
        last_nose_action_ok = false;
        RCLCPP_ERROR(this->get_logger(), "Unable to send nose color to the Arduino.");
    } else {
        last_nose_action_ok = true;
        RCLCPP_DEBUG(this->get_logger(), "Nose color %d sent to Arduino.", msg->color);
    }
}

void NoseController::testNoseLedsCallback(
    const std::shared_ptr<qbo_msgs::srv::TestLeds::Request>,
    std::shared_ptr<qbo_msgs::srv::TestLeds::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "🚦 Starting LED test sequence");

    int code = driver_->testNose();
    if (code < 0) {
        last_nose_action_ok = false;
        RCLCPP_WARN(this->get_logger(), "⚠️ testNose sent, but no response received (as expected)");
        res->success = true;  // ✅ car l'action a été envoyée avec succès
        res->message = "Nose test sent, no response expected";
    } else {
        last_nose_action_ok = true;
        res->success = true;
        res->message = "Nose test executed successfully";
    }
}

void NoseController::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    static const std::array<std::string, 8> color_names = {
        "Off",
        "Red",
        "Blue",
        "Purple",
        "Green",
        "Yellow",
        "Magenta",
        "White"
    };

    if (last_nose_action_ok) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                     "Nose controller operational");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "Nose controller command/test failed");
    }

    stat.add("color_code", static_cast<int>(last_color_));
    stat.add("color_name", color_names[last_color_]);
}
