#include "qbo_arduqbo/qbo_arduqbo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"


QboArduqboManager::QboArduqboManager(std::shared_ptr<rclcpp::Node> node,
                                    const rclcpp::NodeOptions& options,
                                    const std::string &port1,
                                    const std::string &port2)
: node_(node), node_options_(options), port1_(port1), port2_(port2)
{
    // ...
}

void QboArduqboManager::setup() {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
    auto qboard1_id_diag = std::make_shared<int>(-1);
    auto qboard1_version_diag = std::make_shared<int>(-1);
    auto qboard2_id_diag = std::make_shared<int>(-1);
    auto qboard2_version_diag = std::make_shared<int>(-1);

    updater_->setHardwareID("System");

    // =====================
    // ⚙️  Paramètres globaux
    // =====================
    node_->get_parameter("baud1", baud1_);
    node_->get_parameter("baud2", baud2_);
    node_->get_parameter("timeout1", timeout1_);
    node_->get_parameter("timeout2", timeout2_);

    node_->get_parameter("enable_qboard1", enable_qboard1_);
    node_->get_parameter("enable_qboard2", enable_qboard2_);

    node_->get_parameter("enable_battery", enable_battery_);
    node_->get_parameter("enable_base", enable_base_);
    node_->get_parameter("enable_imu_base", enable_imu_base_);
    node_->get_parameter("enable_lcd", enable_lcd_);
    node_->get_parameter("enable_nose", enable_nose_);
    node_->get_parameter("enable_mouth", enable_mouth_);
    node_->get_parameter("enable_audio", enable_audio_);
    node_->get_parameter("enable_sensors", enable_sensors_);

    RCLCPP_INFO(node_->get_logger(), "PORT1: %s (%s)", port1_.c_str(), enable_qboard1_ ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "PORT2: %s (%s)", port2_.c_str(), enable_qboard2_ ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "BAUD: %d / %d | TIMEOUT: %.2f / %.2f", baud1_, baud2_, timeout1_, timeout2_);

    updater_->add("Arduqbo Status", [this, qboard1_id_diag, qboard1_version_diag, qboard2_id_diag, qboard2_version_diag](diagnostic_updater::DiagnosticStatusWrapper &status) {
        const bool qboard1_started = !enable_qboard1_ || (*qboard1_id_diag >= 0 && *qboard1_version_diag >= 0);
        const bool qboard2_started = !enable_qboard2_ || (*qboard2_id_diag >= 0 && *qboard2_version_diag >= 0);

        if (!qboard1_started && !qboard2_started) {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Arduqbo: QBoard1 and QBoard2 failed to start");
        } else if (!qboard1_started) {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Arduqbo: QBoard1 failed to start");
        } else if (!qboard2_started) {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Arduqbo: QBoard2 failed to start");
        } else {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Arduqbo operational");
        }

        status.add("_QBoard1", enable_qboard1_ ? "Enabled" : "Disabled");
        status.add("_QBoard1 ID", *qboard1_id_diag >= 0 ? std::to_string(*qboard1_id_diag) : "N/A");
        status.add("_QBoard1 Version", *qboard1_version_diag >= 0 ? std::to_string(*qboard1_version_diag) : "N/A");
        status.add("_Controller Battery", enable_battery_ ? "Enabled" : "Disabled");
        status.add("_Controller Base", enable_base_ ? "Enabled" : "Disabled");
        status.add("_Controller IMU", enable_imu_base_ ? "Enabled" : "Disabled");
        status.add("_Controller LCD", enable_lcd_ ? "Enabled" : "Disabled");
        status.add("_QBoard2", enable_qboard2_ ? "Enabled" : "Disabled");
        status.add("_QBoard2 ID", *qboard2_id_diag >= 0 ? std::to_string(*qboard2_id_diag) : "N/A");
        status.add("_QBoard2 Version", *qboard2_version_diag >= 0 ? std::to_string(*qboard2_version_diag) : "N/A");
        status.add("_Controller Nose", enable_nose_ ? "Enabled" : "Disabled");
        status.add("_Controller Mouth", enable_mouth_ ? "Enabled" : "Disabled");
        status.add("_Controller Audio", enable_audio_ ? "Enabled" : "Disabled");
        status.add("_Controller Sensors", enable_sensors_ ? "Enabled" : "Disabled");
    });

    // =====================
    // 🔌 Initialisation driver
    // =====================
    arduino_driver_ = std::make_shared<QboDuinoDriver>(port1_, baud1_, port2_, baud2_, timeout1_, timeout2_);

    // =====================
    // 🧩 Chargement des contrôleurs
    // =====================

    // ➕ Qboard 1 (Serial - base)
    if (enable_qboard1_) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo base board communication active");

        // QBoard1 (Base)
        int code1 = arduino_driver_->getVersion("base", board_id, version);
        if (code1 >= 0 && id == 0) {
            *qboard1_id_diag = board_id;
            *qboard1_version_diag = version;
            qboard1_version_ = version;
            RCLCPP_INFO(node_->get_logger(), "      QBoard1 detected — ID: %d, Version: %d", board_id, version);
        } else {
            RCLCPP_WARN(node_->get_logger(), "❌ Failed to get QBoard1 (base) version");
        }

        bool loaded = false;
        if (enable_battery_) {
            auto battery_ctrl = std::make_shared<CBatteryController>(arduino_driver_, node_options_);
            controllers_.push_back(battery_ctrl);
            loaded = true;
        }
        logControllerStatus("Battery", enable_battery_, loaded);

        loaded = false;
        if (enable_base_) {
            auto base_ctrl = std::make_shared<BaseController>(arduino_driver_, node_options_);
            controllers_.push_back(base_ctrl);
            loaded = true;
        }
        logControllerStatus("Base", enable_base_, loaded);

        loaded = false;
        if (enable_imu_base_) {
            auto imu_ctrl = std::make_shared<ImuController>(arduino_driver_,  node_options_);
            controllers_.push_back(imu_ctrl);
            loaded = true;
        }
        logControllerStatus("IMU base", enable_nose_, loaded);

        loaded = false;
        if (enable_lcd_) {
            auto lcd_ctrl = std::make_shared<LcdController>(arduino_driver_,  node_options_);
            controllers_.push_back(lcd_ctrl);
            loaded = true;
        }
        logControllerStatus("LCD", enable_lcd_, loaded);

        loaded = false;
        if (enable_sensors_) {
            auto sens_ctrl = std::make_shared<SensorController>(arduino_driver_, node_options_);
            controllers_.push_back(sens_ctrl);
            loaded = true;
        }
        logControllerStatus("Sensors", enable_sensors_, loaded);


    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Base board communication disabled by config");
    }

    // ➕ Qboard 2 (Serial - head)
    if (enable_qboard2_) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo head board communication active");

        // QBoard2 (Head)
        int code2 = arduino_driver_->getVersion("head", board_id, version);
        if (code2 >= 0 && id == 0) {
            *qboard2_id_diag = board_id;
            *qboard2_version_diag = version;
            qboard2_version_ = version;
            RCLCPP_INFO(node_->get_logger(), "      QBoard2 detected — ID: %d, Version: %d", board_id, version);
        } else {
            RCLCPP_WARN(node_->get_logger(), "❌ Failed to get QBoard2 (head) version");
        }

        bool loaded = false;
        if (enable_nose_) {
            auto nose_ctrl = std::make_shared<NoseController>(arduino_driver_,  node_options_);
            controllers_.push_back(nose_ctrl);
            loaded = true;
        }
        logControllerStatus("Nose", enable_nose_, loaded);

        loaded = false;
        if (enable_mouth_) {
            auto mouth_ctrl = std::make_shared<MouthController>(arduino_driver_,  node_options_);
            controllers_.push_back(mouth_ctrl);
            loaded = true;
        }
        logControllerStatus("Mouth", enable_mouth_, loaded);

        loaded = false;
        if (enable_audio_) {
            auto audio_ctrl = std::make_shared<AudioController>(arduino_driver_,  node_options_);
            controllers_.push_back(audio_ctrl);
            loaded = true;
        }
        logControllerStatus("Audio", enable_audio_, loaded);

    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Head board communication disabled by config");
    }

    // =====================
    // 🚀 Ajout au scheduler
    // =====================
    executor_->add_node(node_);
    for (auto& ctrl : controllers_) {
        executor_->add_node(ctrl);
    }
}


void QboArduqboManager::run() {
    executor_->spin();
}


void QboArduqboManager::logControllerStatus(const std::string &name, bool enabled, bool loaded) {
    if (!enabled) {
        RCLCPP_INFO(node_->get_logger(), "⏹️ %s controller disabled by config", name.c_str());
    } else if (loaded) {
        return;
    } else {
        RCLCPP_WARN(node_->get_logger(), "⚠️ %s controller not loaded", name.c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("qbo_arduqbo", options);
    RCLCPP_INFO(node->get_logger(), "🎬 Starting qbo_arduqbo node");
    try
    {
        std::string port1 = "";
        std::string port2 = "";

        // Récupération des valeurs
        node->get_parameter("port1", port1);
        node->get_parameter("port2", port2);

        if (port1.empty()) {
            RCLCPP_FATAL(node->get_logger(), "❌ USB port for QBoard1 is not defined (key: qbo_arduqbo.port1)");
            return 1;
        }

        if (port2.empty()) {
            RCLCPP_FATAL(node->get_logger(), "❌ USB port for QBoard2 is not defined (key: qbo_arduqbo.port2)");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "✅ Initial configuration validated, launching qbo_arduqbo...");

        QboArduqboManager manager(node, options, port1, port2);

        manager.setup();  // configure les contrôleurs
        manager.run();    // lance l'exécuteur avec tous les nœuds enregistrés
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(node->get_logger(), "🛑 Fatal exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

