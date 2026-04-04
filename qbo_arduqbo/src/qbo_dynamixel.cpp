#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/controllers/dynamixel_hardware.hpp"
#include "qbo_msgs/srv/torque_enable.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>
#include <cmath>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("dynamixel_hardware");
    RCLCPP_INFO(node->get_logger(), "Starting qbo_dynamixel node");

    // ── Paramètres généraux ──────────────────────────────────────────────────
    node->declare_parameter("dynamixel.usb_port",         "/dev/ttyDmx");
    node->declare_parameter("dynamixel.baud_rate",        1000000);
    node->declare_parameter("dynamixel.protocol_version", 1.0);
    node->declare_parameter("dynamixel.motor_keys",       std::vector<std::string>{});
    node->declare_parameter("auto_torque_off",            false);
    node->declare_parameter("auto_torque_off_timeout",    20.0);
    node->declare_parameter("dynamixel_state_rate_hz",    30.0);  // cadence lecture hardware
    node->declare_parameter("dynamixel_joint_rate_hz",    30.0);  // cadence publication /joint_states

    // ── Paramètres diagnostics ────────────────────────────────────────────────
    node->declare_parameter("diagnostic_temp_warn",      60.0);
    node->declare_parameter("diagnostic_voltage_min",     8.0);
    node->declare_parameter("diagnostic_voltage_max",    12.5);
    node->declare_parameter("diagnostic_position_error", 20);

    std::string port     = node->get_parameter("dynamixel.usb_port").as_string();
    int         baud     = node->get_parameter("dynamixel.baud_rate").as_int();
    double      protocol = node->get_parameter("dynamixel.protocol_version").as_double();
    auto        keys     = node->get_parameter("dynamixel.motor_keys").as_string_array();
    bool        auto_torque_off    = node->get_parameter("auto_torque_off").as_bool();
    double      torque_off_timeout = node->get_parameter("auto_torque_off_timeout").as_double();
    double      state_rate         = node->get_parameter("dynamixel_state_rate_hz").as_double();
    double      joint_rate         = node->get_parameter("dynamixel_joint_rate_hz").as_double();
    double      diag_temp_warn     = node->get_parameter("diagnostic_temp_warn").as_double();
    double      diag_volt_min      = node->get_parameter("diagnostic_voltage_min").as_double();
    double      diag_volt_max      = node->get_parameter("diagnostic_voltage_max").as_double();
    int         diag_pos_err       = node->get_parameter("diagnostic_position_error").as_int();

    auto last_cmd_time = node->get_clock()->now();

    RCLCPP_INFO(node->get_logger(),
        "Auto torque off: %s (timeout=%.0fs) | read=%.0fHz joint_pub=%.0fHz",
        auto_torque_off ? "ON" : "OFF", torque_off_timeout, state_rate, joint_rate);
    RCLCPP_INFO(node->get_logger(),
        "Diag: temp_warn=%.0f°C volt=[%.1f-%.1fV] pos_err=%d ticks",
        diag_temp_warn, diag_volt_min, diag_volt_max, diag_pos_err);

    // ── Init hardware ────────────────────────────────────────────────────────
    auto hardware = std::make_shared<DynamixelHardware>();

    hardware_interface::HardwareInfo info;
    info.hardware_parameters["port"]             = port;
    info.hardware_parameters["baud_rate"]        = std::to_string(baud);
    info.hardware_parameters["protocol_version"] = std::to_string(protocol);

    for (const auto & key : keys) {
        std::string base = "dynamixel.motors." + key;

        node->declare_parameter(base + ".name",              "");
        node->declare_parameter(base + ".id",                0);
        node->declare_parameter(base + ".neutral",           512);
        node->declare_parameter(base + ".ticks",             1024);
        node->declare_parameter(base + ".range",             300.0);
        node->declare_parameter(base + ".torque_limit",      200);
        node->declare_parameter(base + ".invert",            false);
        node->declare_parameter(base + ".max_speed",         1.0);
        node->declare_parameter(base + ".min_angle_degrees", -70.0);
        node->declare_parameter(base + ".max_angle_degrees",  70.0);

        hardware_interface::ComponentInfo j;
        j.name                       = node->get_parameter(base + ".name").as_string();
        j.parameters["id"]           = std::to_string(node->get_parameter(base + ".id").as_int());
        j.parameters["neutral"]      = std::to_string(node->get_parameter(base + ".neutral").as_int());
        j.parameters["ticks"]        = std::to_string(node->get_parameter(base + ".ticks").as_int());
        j.parameters["range"]        = std::to_string(node->get_parameter(base + ".range").as_double());
        j.parameters["torque_limit"] = std::to_string(node->get_parameter(base + ".torque_limit").as_int());
        j.parameters["invert"]       = node->get_parameter(base + ".invert").as_bool() ? "true" : "false";
        j.parameters["max_speed"]    = std::to_string(node->get_parameter(base + ".max_speed").as_double());
        j.parameters["min_angle"]    = std::to_string(
            node->get_parameter(base + ".min_angle_degrees").as_double() * M_PI / 180.0);
        j.parameters["max_angle"]    = std::to_string(
            node->get_parameter(base + ".max_angle_degrees").as_double() * M_PI / 180.0);

        info.joints.push_back(j);
        RCLCPP_INFO(node->get_logger(),
            "Loaded joint %s (ID %s) range=%.0f° min=%.2f max=%.2f rad max_speed=%.1f rad/s",
            j.name.c_str(), j.parameters["id"].c_str(),
            node->get_parameter(base + ".range").as_double(),
            std::stod(j.parameters["min_angle"]),
            std::stod(j.parameters["max_angle"]),
            std::stod(j.parameters["max_speed"]));
    }

    if (hardware->on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize hardware");
        return 1;
    }

    // ── Diagnostics sur le node principal ────────────────────────────────────
    // Le Updater possède un timer interne → spin_some(node) dans la boucle suffit.
    // Pas de diag_node_ séparé → pas de segfault.
    auto diag_updater = std::make_shared<diagnostic_updater::Updater>(node);
    diag_updater->setHardwareID("DynamixelAX12");

    for (size_t i = 0; i < hardware->getServos().size(); i++) {
        const std::string servo_name = hardware->getServos()[i].name;
        diag_updater->add("Servo " + servo_name,
            [&hardware, i, diag_temp_warn, diag_volt_min, diag_volt_max, diag_pos_err](
                diagnostic_updater::DiagnosticStatusWrapper & stat)
            {
                const auto & s = hardware->getServos()[i];
                stat.add("Position (rad)",   s.position);
                stat.add("Velocity (rad/s)", s.velocity);
                stat.add("Torque load",      s.torque_load);
                stat.add("Temperature (C)",  s.temperature);
                stat.add("Torque enabled",   s.torque_enabled ? "true" : "false");
                // Voltage et position_error non lus directement (pas dans read()),
                // les seuils sont tracés pour référence future
                (void)diag_volt_min; (void)diag_volt_max; (void)diag_pos_err;

                if      (s.temperature >= 70.0)
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature critique !");
                else if (s.temperature >= diag_temp_warn)
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,  "Temperature elevee");
                else
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,    "OK");
            });
    }

    // ── Publisher /joint_states ───────────────────────────────────────────────
    auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    // ── Subscriber /cmd_joints ────────────────────────────────────────────────
    auto cmd_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/cmd_joints", 10,
        [&hardware, &node, &last_cmd_time](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            auto & wb           = hardware->getWorkbench();
            const auto & servos = hardware->getServos();

            last_cmd_time = node->get_clock()->now();

            // Réactive le torque si éteint par timeout
            for (auto & s : hardware->getServos()) {
                if (!s.torque_enabled) {
                    hardware->getWorkbench().torqueOn(s.id);
                    s.torque_enabled = true;
                    RCLCPP_INFO(node->get_logger(),
                        "Auto torque ON (commande reçue) : %s", s.name.c_str());
                }
            }

            for (size_t i = 0; i < msg->name.size(); i++) {
                for (const auto & s : servos) {
                    if (s.name != msg->name[i] || i >= msg->position.size()) continue;

                    uint8_t sid  = static_cast<uint8_t>(s.id);
                    int32_t goal = hardware->angleToTicksPublic(s, msg->position[i]);

                    // Vitesse clampée à max_speed du yaml
                    if (i < msg->velocity.size() && msg->velocity[i] > 0.0) {
                        double vel      = std::min(msg->velocity[i], s.max_speed);
                        int speed_ticks = hardware->radPerSecToDxlSpeed(vel);

                        RCLCPP_INFO(node->get_logger(),
                            "Moving_Speed servo %d -> %.2f rad/s (req=%.1f max=%.1f) -> %d ticks",
                            sid, vel, msg->velocity[i], s.max_speed, speed_ticks);

                        const char * log = nullptr;
                        if (!wb.itemWrite(sid, "Moving_Speed", speed_ticks, &log))
                            RCLCPP_WARN(node->get_logger(),
                                "Moving_Speed failed servo %d: %s", sid, log ? log : "unknown");
                    }

                    RCLCPP_INFO(node->get_logger(),
                        "cmd %s -> %.3f rad -> tick %d", s.name.c_str(), msg->position[i], goal);

                    const char * log = nullptr;
                    if (!wb.itemWrite(sid, "Goal_Position", goal, &log))
                        RCLCPP_WARN(node->get_logger(),
                            "Goal_Position failed servo %d: %s", sid, log ? log : "unknown");
                    break;
                }
            }
        }
    );

    // ── Services torque_enable par servo ─────────────────────────────────────
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    for (const auto & s : hardware->getServos()) {
        auto svc = node->create_service<qbo_msgs::srv::TorqueEnable>(
            "/" + s.name + "/torque_enable",
            [&hardware, id = s.id, name = s.name](
                const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request>  req,
                      std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res)
            {
                auto & wb = hardware->getWorkbench();
                if (req->torque_enable) {
                    wb.torqueOn(id);
                } else {
                    wb.torqueOff(id);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                for (auto & sv : hardware->getServos()) {
                    if (sv.id == id) { sv.torque_enabled = req->torque_enable; break; }
                }
                std::string msg_str = name + " torque -> " + (req->torque_enable ? "ON" : "OFF");
                res->success = true;
                res->message = msg_str;
                RCLCPP_INFO(rclcpp::get_logger("qbo_dynamixel"), "%s", msg_str.c_str());
            }
        );
        services.push_back(svc);
    }

    RCLCPP_INFO(node->get_logger(), "Hardware initialized. Starting read loop...");

    // Deux compteurs pour découpler lecture hardware et publication /joint_states
    rclcpp::WallRate rate(state_rate);
    const int joint_pub_every = std::max(1, static_cast<int>(std::round(state_rate / joint_rate)));
    int cycle = 0;

    // ── Boucle principale ─────────────────────────────────────────────────────
    while (rclcpp::ok()) {

        if (hardware->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0)) ==
            hardware_interface::return_type::OK)
        {
            // Auto torque off après inactivité
            if (auto_torque_off) {
                double elapsed = (node->get_clock()->now() - last_cmd_time).seconds();
                if (elapsed > torque_off_timeout) {
                    for (auto & s : hardware->getServos()) {
                        if (s.torque_enabled) {
                            hardware->getWorkbench().torqueOff(s.id);
                            s.torque_enabled = false;
                            RCLCPP_WARN(node->get_logger(),
                                "Auto torque OFF : %s (inactif depuis %.0fs)",
                                s.name.c_str(), elapsed);
                        }
                    }
                }
            }

            // Publication /joint_states à joint_rate Hz (découpée de state_rate)
            if (++cycle >= joint_pub_every) {
                cycle = 0;

                const auto & servos = hardware->getServos();
                sensor_msgs::msg::JointState js_msg;
                js_msg.header.stamp = node->get_clock()->now();
                js_msg.name.reserve(servos.size());
                js_msg.position.reserve(servos.size());
                js_msg.velocity.reserve(servos.size());
                js_msg.effort.reserve(servos.size());

                for (const auto & s : servos) {
                    js_msg.name.push_back(s.name);
                    js_msg.position.push_back(s.position);
                    js_msg.velocity.push_back(s.velocity);
                    js_msg.effort.push_back(s.effort);

                    RCLCPP_INFO(node->get_logger(),
                        "Motor %s (ID %d): pos=%.3f rad  temp=%.0f°C  torque=%s",
                        s.name.c_str(), s.id, s.position, s.temperature,
                        s.torque_enabled ? "ON" : "OFF");
                }

                joint_state_pub->publish(js_msg);
            }
        }

        // spin_some gère subscribers, services ET le timer interne du diag_updater
        // → publication automatique sur /diagnostics sans appel supplémentaire
        rclcpp::spin_some(node);
        rate.sleep();
    }

    hardware->on_deactivate(rclcpp_lifecycle::State());
    rclcpp::shutdown();
    return 0;
}