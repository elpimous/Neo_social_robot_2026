#ifndef DYNAMIXEL_CONTROLLER_HPP
#define DYNAMIXEL_CONTROLLER_HPP

#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <qbo_msgs/srv/torque_enable.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

inline double radians(double angle) {
    return angle * M_PI / 180.0;
}

class DynamixelServo
{
public:
    DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
               const std::string & name,
               DynamixelWorkbench* wb);

    ~DynamixelServo();

    void setAngle(float ang, float velocity = 1.0f);
    float getAngle();

    bool servoTorqueEnable(
        const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
        std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res);

    void setParams(const std::string & motor_key);

    // 🆕 Suivi de l’état du couple
    bool isTorqueEnabled() const { return torque_enabled_; }
    void setTorqueEnabled(bool enabled) { torque_enabled_ = enabled; }

    // 🆕 Nom utile pour les logs
    std::string getName() const { return name_; }

    int id_;
    bool invert_;
    float angle_;
    int neutral_;
    int ticks_;
    float max_angle_;
    float min_angle_;
    float rad_per_tick_;
    float max_speed_;
    float range_;
    uint16_t model_number_;
    int torque_limit_;

    std::string name_;
    std::string joint_name_;

    rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

protected:
    std::shared_ptr<rclcpp::Node> node_;
    DynamixelWorkbench* dxl_wb_;
    rclcpp::Service<qbo_msgs::srv::TorqueEnable>::SharedPtr servo_torque_enable_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 🆕 état du couple (géré en interne)
    bool torque_enabled_ = true;
};

class DynamixelController
{
public:
    explicit DynamixelController(const std::shared_ptr<rclcpp::Node> & node);
    ~DynamixelController();

private:
    void publishJointStates();
    void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void checkInactivity();  // 🆕 gestion de l'inactivité
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & parameters);

    std::shared_ptr<rclcpp::Node> node_;
    std::string usb_port_;
    int baud_rate_;
    double protocol_version_;
    std::vector<std::unique_ptr<DynamixelServo>> servos_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;

    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr inactivity_timer_;
    rclcpp::Clock steady_clock_{RCL_SYSTEM_TIME};  // horloge système stable
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr controller_param_callback_handle_;

    rclcpp::Time last_cmd_time_;
    bool auto_torque_off_ = true;
    double timeout_sec_ = 2.0;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    DynamixelWorkbench dxl_wb_;
    std::shared_ptr<diagnostic_updater::Updater> diagnostics_;
};

#endif  // DYNAMIXEL_CONTROLLER_HPP
