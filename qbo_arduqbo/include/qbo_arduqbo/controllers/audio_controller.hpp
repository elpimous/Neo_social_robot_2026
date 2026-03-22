#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"
#include "qbo_msgs/msg/mic_report.hpp"
#include "qbo_msgs/srv/set_mouth_animation.hpp"

class AudioController : public rclcpp::Node
{
public:
    AudioController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options);

private:
    void publishMicReport();
    void setMouthAnimationCallback(
        const std::shared_ptr<qbo_msgs::srv::SetMouthAnimation::Request>,
        std::shared_ptr<qbo_msgs::srv::SetMouthAnimation::Response>);

    std::shared_ptr<QboDuinoDriver> driver_;
    rclcpp::Publisher<qbo_msgs::msg::MicReport>::SharedPtr mic_pub_;
    rclcpp::Service<qbo_msgs::srv::SetMouthAnimation>::SharedPtr mouth_anim_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string mic_topic_;
    double rate_;
    bool mouth_animation_enabled_ = false;

};
