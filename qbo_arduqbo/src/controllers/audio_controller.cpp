#include "qbo_arduqbo/controllers/audio_controller.hpp"
#include <rclcpp/rclcpp.hpp>

AudioController::AudioController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("audio_ctrl", "qbo_arduqbo", options), driver_(driver)
{
    // Lecture des paramètres
    this->get_parameter("topic", mic_topic_);
    this->get_parameter("rate", rate_);

    mic_pub_ = this->create_publisher<qbo_msgs::msg::MicReport>(mic_topic_, 10);

    mouth_anim_srv_ = this->create_service<qbo_msgs::srv::SetMouthAnimation>(
        this->get_name() + std::string("/set_mouth_animation"),
        std::bind(&AudioController::setMouthAnimationCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        std::bind(&AudioController::publishMicReport, this)
    );

    RCLCPP_INFO(this->get_logger(), "✅ AudioController initialized with:\n"
                                "       - Rate: %.2f Hz\n"
                                "       - Mic topic: %s",
            rate_, mic_topic_.c_str());
}

void AudioController::publishMicReport()
{
    int16_t ambient;
    uint8_t direction;
    uint16_t m0, m1, m2;

    if (driver_->getMicReport(ambient, direction, m0, m1, m2) < 0) {
        RCLCPP_WARN(this->get_logger(), "❌ Failed to retrieve mic report from Arduino");
        return;
    }

    qbo_msgs::msg::MicReport msg;
    msg.ambient_noise = ambient;
    msg.sound_direction = direction;
    msg.mic_levels[0] = static_cast<int16_t>(m0);
    msg.mic_levels[1] = static_cast<int16_t>(m1);
    msg.mic_levels[2] = static_cast<int16_t>(m2);
    msg.mouth_animation_enabled = mouth_animation_enabled_;

    mic_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "MicReport: amb=%d, dir=%u, mic=[%d, %d, %d]",
    //          ambient, direction, m0, m1, m2);
}


void AudioController::setMouthAnimationCallback(
    const std::shared_ptr<qbo_msgs::srv::SetMouthAnimation::Request> req,
    std::shared_ptr<qbo_msgs::srv::SetMouthAnimation::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "🗣️ Set mouth animation: %s", req->enable ? "ON" : "OFF");

    int code = driver_->setMouthAnimation(req->enable);
    if (code < 0) {
        res->success = false;
        res->message = "Failed to send command to Arduino";
        mouth_animation_enabled_ = req->enable;
    } else {
        res->success = true;
        res->message = "Command sent successfully";
    }
}
