#include "qbo_arduqbo/controllers/dynamixel_hardware.hpp"
#include <cmath>
#include <algorithm>

hardware_interface::CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Initializing Dynamixel Hardware Interface...");

    if (info.hardware_parameters.empty()) {
        RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"), "No hardware parameters provided!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    port_ = info.hardware_parameters.at("port");
    baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
    protocol_version_ = std::stod(info.hardware_parameters.at("protocol_version"));

    if (!dxl_wb_.init(port_.c_str(), baud_rate_)) {
        RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"), "Failed to init DynamixelWorkbench on %s @ %d bps", port_.c_str(), baud_rate_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    dxl_wb_.setPacketHandler(protocol_version_);

    servos_.clear();
    for (const auto & joint : info.joints) {
        Servo s;
        s.name = joint.name;
        s.id   = std::stoi(joint.parameters.at("id"));
        s.neutral = std::stoi(joint.parameters.at("neutral"));
        s.ticks   = std::stoi(joint.parameters.at("ticks"));
        s.rad_per_tick = M_PI / (s.ticks / 2.0);
        s.torque_limit = std::stoi(joint.parameters.at("torque_limit"));
        s.invert = (joint.parameters.at("invert") == "true");
        s.position = 0.0;
        s.velocity = 0.0;
        s.effort   = 0.0;
        s.command  = 0.0;

        uint16_t model_number = 0;
        if (!dxl_wb_.ping(s.id, &model_number)) {
            RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"),
                "Cannot ping servo ID %d - verifier ID, baud rate et cablage", s.id);
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
            "Servo ID %d detecte, modele : %d", s.id, model_number);

        dxl_wb_.itemWrite(s.id, "Torque_Limit", s.torque_limit);
        dxl_wb_.torqueOn(s.id);

        servos_.push_back(s);

        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
            "Servo %s (ID %d) configure - neutral=%d, invert=%d",
            s.name.c_str(), s.id, s.neutral, s.invert);
    }

    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "DynamixelHardware initialized with %zu servos", servos_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Configuring Dynamixel Hardware...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Activating Dynamixel Hardware - Torque already ON");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Deactivating - Turning torque OFF");
    for (auto & s : servos_) {
        dxl_wb_.torqueOff(s.id);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto & s : servos_) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(s.name, "position", &s.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(s.name, "velocity", &s.velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(s.name, "effort",   &s.effort));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto & s : servos_) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(s.name, "position", &s.command));
    }
    return command_interfaces;
}

hardware_interface::return_type DynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto & s : servos_) {
        int32_t pos = 0;
        if (!dxl_wb_.itemRead(s.id, "Present_Position", &pos)) {
            RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"), "Failed to read position from servo %d", s.id);
            return hardware_interface::return_type::ERROR;
        }
        s.position = ticksToAngle(s, pos);
        s.velocity = 0.0;
        s.effort   = 0.0;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto & s : servos_) {
        int goal = angleToTicks(s, s.command);
        if (!dxl_wb_.itemWrite(s.id, "Goal_Position", goal)) {
            RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"), "Failed to write goal position to servo %d", s.id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

int DynamixelHardware::angleToTicks(const Servo & servo, double rad)
{
    double clamped = std::clamp(rad, -M_PI, M_PI);
    if (servo.invert) clamped = -clamped;
    return static_cast<int>(std::round(clamped / servo.rad_per_tick)) + servo.neutral;
}

double DynamixelHardware::ticksToAngle(const Servo & servo, int ticks)
{
    double angle = (ticks - servo.neutral) * servo.rad_per_tick;
    if (servo.invert) angle = -angle;
    return angle;
}
