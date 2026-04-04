#include "qbo_arduqbo/controllers/dynamixel_hardware.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

// Conversion vitesse AX-12A : 0.111 rpm/unit ≈ 0.0116 rad/s
constexpr double AX12A_SPEED_RAD_PER_UNIT = 0.0116;

////////////////////////////////////////////////////////////////////////////////
// Lifecycle : initialisation du hardware
////////////////////////////////////////////////////////////////////////////////
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

    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
        "Opening port %s @ %d bps", port_.c_str(), baud_rate_);

    if (!dxl_wb_.init(port_.c_str(), baud_rate_)) {
        RCLCPP_FATAL(rclcpp::get_logger("DynamixelHardware"),
            "Failed to init DynamixelWorkbench on %s @ %d bps", port_.c_str(), baud_rate_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    dxl_wb_.setPacketHandler(protocol_version_);

    servos_.clear();

    for (const auto & joint : info.joints) {
        Servo s;
        s.name        = joint.name;
        s.id          = std::stoi(joint.parameters.at("id"));
        s.neutral     = std::stoi(joint.parameters.at("neutral"));
        s.ticks       = std::stoi(joint.parameters.at("ticks"));
        s.torque_limit = std::stoi(joint.parameters.at("torque_limit"));
        s.invert      = (joint.parameters.at("invert") == "true");
        s.max_speed   = joint.parameters.count("max_speed") ? std::stod(joint.parameters.at("max_speed")) : 1.0;
        s.min_angle   = joint.parameters.count("min_angle") ? std::stod(joint.parameters.at("min_angle")) : -1.22;
        s.max_angle   = joint.parameters.count("max_angle") ? std::stod(joint.parameters.at("max_angle")) :  1.22;

        // ✅ rad_per_tick calculé depuis la plage mécanique réelle (300° pour AX-12A)
        // Ancienne formule : M_PI / (ticks/2) supposait 360° → erreur de 20%
        s.range_deg   = joint.parameters.count("range") ? std::stod(joint.parameters.at("range")) : 300.0;
        s.rad_per_tick = (s.range_deg * M_PI / 180.0) / static_cast<double>(s.ticks);

        s.position      = 0.0;
        s.velocity      = 0.0;
        s.effort        = 0.0;
        s.command       = 0.0;
        s.temperature   = 0.0;
        s.torque_load   = 0.0;
        s.torque_enabled = false;

        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
            "Pinging servo ID %d (%s)...", s.id, s.name.c_str());

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
            "Servo ID %d detecte : modele=%d  range=%.0f deg  rad_per_tick=%.6f",
            s.id, model_number, s.range_deg, s.rad_per_tick);

        servos_.push_back(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Configuration : torque limit, torque ON, centrage à vitesse réduite
    for (auto & s : servos_) {
        dxl_wb_.itemWrite(s.id, "Torque_Limit", s.torque_limit);
        dxl_wb_.torqueOn(s.id);
        s.torque_enabled = true;

        int init_speed = radPerSecToDxlSpeed(0.5);
        dxl_wb_.itemWrite(s.id, "Moving_Speed", init_speed);
        dxl_wb_.itemWrite(s.id, "Goal_Position", s.neutral);

        RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
            "Servo %s (ID %d) configure et centre - neutral=%d invert=%d torque_limit=%d",
            s.name.c_str(), s.id, s.neutral, s.invert, s.torque_limit);
    }

    RCLCPP_INFO(rclcpp::get_logger("DynamixelHardware"),
        "DynamixelHardware initialized with %zu servos", servos_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// Conversion vitesse rad/s → Dynamixel units
////////////////////////////////////////////////////////////////////////////////
int DynamixelHardware::radPerSecToDxlSpeed(double rad_s)
{
    const double max_rad_s = 1023.0 * AX12A_SPEED_RAD_PER_UNIT;
    double clamped = std::clamp(rad_s, 0.0, max_rad_s);
    return static_cast<int>(std::round(clamped / AX12A_SPEED_RAD_PER_UNIT));
}

////////////////////////////////////////////////////////////////////////////////
// Lifecycle callbacks simples
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// Export interfaces ROS2 control
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// Lecture capteurs
////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type DynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto & s : servos_) {
        int32_t pos = 0, vel = 0, load = 0, temp = 0, torque_en = 0;

        if (!dxl_wb_.itemRead(s.id, "Present_Position", &pos)) {
            RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"),
                "Cannot read servo %d, keeping last values", s.id);
            continue;
        }

        dxl_wb_.itemRead(s.id, "Present_Speed",       &vel);
        dxl_wb_.itemRead(s.id, "Present_Load",        &load);
        dxl_wb_.itemRead(s.id, "Present_Temperature", &temp);
        dxl_wb_.itemRead(s.id, "Torque_Enable",       &torque_en);

        s.position       = ticksToAngle(s, pos);
        s.velocity       = ((vel  & 0x400) ? -1 : 1) * (vel  & 0x3FF) * AX12A_SPEED_RAD_PER_UNIT;
        s.torque_load    = ((load & 0x400) ? -1 : 1) * (load & 0x3FF);
        s.temperature    = static_cast<double>(temp);
        s.torque_enabled = (torque_en != 0);
        s.effort         = s.torque_load;
    }
    return hardware_interface::return_type::OK;
}

////////////////////////////////////////////////////////////////////////////////
// Écriture commandes
////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type DynamixelHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto & s : servos_) {
        int goal  = angleToTicks(s, s.command);
        int speed = radPerSecToDxlSpeed(s.max_speed);

        if (speed != s.last_speed) {
            if (!dxl_wb_.itemWrite(s.id, "Moving_Speed", speed)) {
                RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"),
                    "Failed to write speed to servo %d", s.id);
                return hardware_interface::return_type::ERROR;
            }
            s.last_speed = speed;
        }

        if (!dxl_wb_.itemWrite(s.id, "Goal_Position", goal)) {
            RCLCPP_WARN(rclcpp::get_logger("DynamixelHardware"),
                "Failed to write goal to servo %d", s.id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

////////////////////////////////////////////////////////////////////////////////
// Conversions angle ↔ ticks
////////////////////////////////////////////////////////////////////////////////
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