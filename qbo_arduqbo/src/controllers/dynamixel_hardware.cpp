#include "qbo_arduqbo/controllers/dynamixel_hardware.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

hardware_interface::CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Initializing Dynamixel Hardware Interface...");

    if (info.hardware_parameters.empty()) {
        RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"), "No hardware parameters provided!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    port_             = info.hardware_parameters.at("port");
    baud_rate_        = std::stoi(info.hardware_parameters.at("baud_rate"));
    protocol_version_ = std::stod(info.hardware_parameters.at("protocol_version"));

    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Opening port %s @ %d bps", port_.c_str(), baud_rate_);

    if (!dxl_wb_.init(port_.c_str(), baud_rate_)) {
        RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"),
            "Failed to init DynamixelWorkbench on %s @ %d bps", port_.c_str(), baud_rate_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    dxl_wb_.setPacketHandler(protocol_version_);

    servos_.clear();

    for (const auto & joint : info.joints) {
        Servo s;
        s.name         = joint.name;
        s.id           = std::stoi(joint.parameters.at("id"));
        s.neutral      = std::stoi(joint.parameters.at("neutral"));
        s.ticks        = std::stoi(joint.parameters.at("ticks"));
        s.rad_per_tick = M_PI / (s.ticks / 2.0);
        s.torque_limit = std::stoi(joint.parameters.at("torque_limit"));
        s.invert       = (joint.parameters.at("invert") == "true");
        s.max_speed    = joint.parameters.count("max_speed") ? std::stod(joint.parameters.at("max_speed")) : 1.0;
        s.min_angle    = joint.parameters.count("min_angle") ? std::stod(joint.parameters.at("min_angle")) : -1.22;
        s.max_angle    = joint.parameters.count("max_angle") ? std::stod(joint.parameters.at("max_angle")) :  1.22;
        s.position     = 0.0; s.velocity = 0.0; s.effort = 0.0; s.command = 0.0;
        s.temperature  = 0.0; s.torque_load = 0.0; s.torque_enabled = false;

        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Pinging servo ID %d...", s.id);

        uint16_t model_number = 0;
        bool found = false;
        for (int attempt = 0; attempt < 5 && !found; attempt++) {
            if (dxl_wb_.ping(s.id, &model_number)) {
                found = true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"),
                    "Ping ID %d echoue (tentative %d/5), retry...", s.id, attempt + 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!found) {
            RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"),
                "Cannot ping servo ID %d apres 5 tentatives", s.id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
            "Servo ID %d detecte, modele : %d", s.id, model_number);

        servos_.push_back(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (auto & s : servos_) {
        dxl_wb_.itemWrite(s.id, "Torque_Limit", s.torque_limit);
        dxl_wb_.torqueOn(s.id);
        s.torque_enabled = true;
        dxl_wb_.itemWrite(s.id, "Goal_Position", s.neutral);
        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
            "Servo %s (ID %d) configure et centre - neutral=%d, invert=%d",
            s.name.c_str(), s.id, s.neutral, s.invert);
    }

    diag_node_    = rclcpp::Node::make_shared("dynamixel_diagnostics");
    diag_updater_ = std::make_shared<diagnostic_updater::Updater>(diag_node_);
    diag_updater_->setHardwareID("DynamixelAX12");
    for (size_t i = 0; i < servos_.size(); i++) {
        diag_updater_->add("Servo " + servos_[i].name,
            [this, i](diagnostic_updater::DiagnosticStatusWrapper & stat) {
                diagnosticCallback(stat, i);
            });
    }

    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
        "DynamixelHardware initialized with %zu servos", servos_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

void DynamixelHardware::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat, int idx)
{
    const Servo & s = servos_[idx];
    stat.add("Position (rad)",   s.position);
    stat.add("Velocity (rad/s)", s.velocity);
    stat.add("Torque load",      s.torque_load);
    stat.add("Temperature (C)",  s.temperature);
    stat.add("Torque enabled",   s.torque_enabled ? "true" : "false");
    if      (s.temperature >= 70.0) stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature critique !");
    else if (s.temperature >= 60.0) stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,  "Temperature elevee");
    else                            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,     "OK");
}

hardware_interface::CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Configuring...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Activating - Torque already ON");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"), "Deactivating - Turning torque OFF");
    for (auto & s : servos_) {
        dxl_wb_.torqueOff(s.id);
        s.torque_enabled = false;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (auto & s : servos_) {
        si.emplace_back(hardware_interface::StateInterface(s.name, "position", &s.position));
        si.emplace_back(hardware_interface::StateInterface(s.name, "velocity", &s.velocity));
        si.emplace_back(hardware_interface::StateInterface(s.name, "effort",   &s.effort));
    }
    return si;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (auto & s : servos_)
        ci.emplace_back(hardware_interface::CommandInterface(s.name, "position", &s.command));
    return ci;
}

hardware_interface::return_type DynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto & s : servos_) {
        int32_t pos = 0, vel = 0, load = 0, temp = 0, torque_en = 0;

        if (!dxl_wb_.itemRead(s.id, "Present_Position", &pos)) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DynamixelHardware"),
                *rclcpp::Clock::make_shared(), 2000,
                "Cannot read servo %d, keeping last values", s.id);
            continue;
        }

        dxl_wb_.itemRead(s.id, "Present_Speed",       &vel);
        dxl_wb_.itemRead(s.id, "Present_Load",        &load);
        dxl_wb_.itemRead(s.id, "Present_Temperature", &temp);
        dxl_wb_.itemRead(s.id, "Torque_Enable",       &torque_en);

        s.position       = ticksToAngle(s, pos);
        s.velocity       = ((vel  & 0x400) ? -1 : 1) * (vel  & 0x3FF) * 0.01194;
        s.torque_load    = ((load & 0x400) ? -1 : 1) * (load & 0x3FF);
        s.temperature    = static_cast<double>(temp);
        s.torque_enabled = (torque_en != 0);
        s.effort         = s.torque_load;
    }

    if (diag_updater_) {
        diag_updater_->force_update();
        rclcpp::spin_some(diag_node_);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto & s : servos_) {
        int goal = angleToTicks(s, s.command);
        if (!dxl_wb_.itemWrite(s.id, "Goal_Position", goal)) {
            RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"),
                "Failed to write goal to servo %d", s.id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

int DynamixelHardware::angleToTicks(const Servo & servo, double rad)
{
    double clamped = std::clamp(rad, servo.min_angle, servo.max_angle);
    if (servo.invert) clamped = -clamped;
    return static_cast<int>(std::round(clamped / servo.rad_per_tick)) + servo.neutral;
}

double DynamixelHardware::ticksToAngle(const Servo & servo, int ticks)
{
    double angle = (ticks - servo.neutral) * servo.rad_per_tick;
    if (servo.invert) angle = -angle;
    return angle;
}
