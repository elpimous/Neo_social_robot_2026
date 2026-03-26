#ifndef DYNAMIXEL_HARDWARE_HPP
#define DYNAMIXEL_HARDWARE_HPP

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    struct Servo {
        std::string name;
        int id;
        double position    = 0.0;
        double velocity    = 0.0;
        double effort      = 0.0;
        double command     = 0.0;
        double temperature = 0.0;
        double torque_load = 0.0;
        bool   torque_enabled = false;

        double rad_per_tick = 0.0;
        int neutral      = 512;
        int ticks        = 1024;
        int torque_limit = 200;
        bool invert      = false;
        double max_speed  = 1.0;
        double min_angle = -1.22;  // rad
        double max_angle =  1.22;  // rad
    };

    DynamixelWorkbench& getWorkbench() { return dxl_wb_; }
    std::vector<Servo>& getServos() { return servos_; }
    const std::vector<Servo>& getServos() const { return servos_; }

    int angleToTicksPublic(const Servo & s, double rad) { return angleToTicks(s, rad); }

private:
    DynamixelWorkbench dxl_wb_;

    std::string port_;
    int baud_rate_            = 1000000;
    double protocol_version_  = 1.0;

    std::vector<Servo> servos_;

    // Diagnostics
    std::shared_ptr<rclcpp::Node>               diag_node_;
    std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat, int servo_index);

    int  angleToTicks(const Servo & servo, double rad);
    double ticksToAngle(const Servo & servo, int ticks);
};

#endif