/**
 * @file    talk_node.cpp
 * @brief   Node ROS2 "talk" — synthèse vocale de Néo (voix axel, VITS).
 *
 * Topics :
 *   sub  /to_speak        std_msgs/String   texte à dire (mis en file)
 *   sub  /interrupt_tts   std_msgs/Empty    coupe immédiatement la parole
 *   sub  /stop_speaking   std_msgs/String   idem (compatibilité ancien node)
 *   pub  /is_speaking     std_msgs/Bool     état parole (sur changement + 1 Hz)
 *
 * Paramètres modifiables à chaud :
 *   volume_gain, speed, sid, alsa_volume_percent
 *     ros2 param set /talk_node volume_gain 1.5
 *
 * Robustesse :
 *   - carte son réglée à alsa_volume_percent (100 %) + unmute au démarrage
 *     et après chaque reconnexion ;
 *   - chien de garde : si la carte USB disparaît, ré-ouverture automatique ;
 *   - process séparé du node listen (lancés avec respawn) : un plantage
 *     de l'un n'arrête jamais l'autre.
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <memory>

#include "listen_and_talk/alsa_mixer.hpp"
#include "listen_and_talk/audio_output.hpp"
#include "listen_and_talk/talk_engine.hpp"
#include "ros_common.hpp"

using namespace std::chrono_literals;

class TalkNode : public rclcpp::Node {
 public:
  TalkNode() : Node("talk_node") {
    // ── Paramètres ──────────────────────────────────────────────────────────
    const auto model = declare_parameter<std::string>("tts_model", "pkg://qbo_tts/models/axel.onnx");
    const auto tokens = declare_parameter<std::string>("tts_tokens", "pkg://qbo_tts/models/tokens.txt");
    const auto data = declare_parameter<std::string>("tts_data_dir", "pkg://qbo_tts/models/espeak-ng-data");
    const auto provider = declare_parameter<std::string>("tts_provider", "cuda");
    const int threads = declare_parameter<int>("tts_num_threads", 2);
    const int sid = declare_parameter<int>("sid", 0);
    const double speed = declare_parameter<double>("speed", 1.0);
    const double gain = declare_parameter<double>("volume_gain", 1.0);
    const bool normalize = declare_parameter<bool>("normalize", true);
    const double target_db = declare_parameter<double>("target_peak_db", -1.0);
    const double pause_ms = declare_parameter<double>("sentence_pause_ms", 140.0);
    const bool warmup = declare_parameter<bool>("warmup", true);
    const auto startup = declare_parameter<std::string>("startup_phrase", "");

    const auto dev = declare_parameter<std::string>("audio_device", "USB Audio Device");
    const int rate = declare_parameter<int>("output_sample_rate", 44100);
    const int chans = declare_parameter<int>("output_channels", 2);
    const double lat_ms = declare_parameter<double>("output_latency_ms", 50.0);
    alsa_card_ = declare_parameter<std::string>("alsa_card", "");
    alsa_pct_ = declare_parameter<int>("alsa_volume_percent", 100);
    if (alsa_card_.empty()) alsa_card_ = dev;

    const auto in_topic = declare_parameter<std::string>("input_topic", "/to_speak");

    auto log = lat::ros_logger(get_logger());

    // ── Volume carte son ────────────────────────────────────────────────────
    apply_alsa_volume();

    // ── Sortie audio ────────────────────────────────────────────────────────
    lat::AudioOutput::Config oc;
    oc.device_hint = dev;
    oc.sample_rate = rate;
    oc.channels = chans;
    oc.latency_s = lat_ms / 1000.0;
    out_ = std::make_unique<lat::AudioOutput>(oc, log);
    if (!out_->open()) RCLCPP_ERROR(get_logger(), "Sortie audio indisponible — nouvel essai automatique");

    // ── Moteur TTS ──────────────────────────────────────────────────────────
    lat::TalkEngine::Config tc;
    tc.model = lat::resolve_path(model);
    tc.tokens = lat::resolve_path(tokens);
    tc.data_dir = lat::resolve_path(data);
    tc.provider = provider;
    tc.num_threads = threads;
    tc.sid = sid;
    tc.speed = static_cast<float>(speed);
    tc.volume_gain = static_cast<float>(gain);
    tc.normalize = normalize;
    tc.target_peak = static_cast<float>(std::pow(10.0, target_db / 20.0));
    tc.sentence_pause_ms = static_cast<float>(pause_ms);
    for (const auto& f : {tc.model, tc.tokens})
      if (!lat::file_exists(f)) RCLCPP_ERROR(get_logger(), "Fichier introuvable : %s", f.c_str());

    tts_ = std::make_unique<lat::TalkEngine>(tc, out_.get(), log);
    if (!tts_->init()) throw std::runtime_error("Échec chargement TTS");
    if (warmup) {
      const auto t0 = std::chrono::steady_clock::now();
      tts_->warmup();
      RCLCPP_INFO(get_logger(), "TTS : warmup %ld ms",
                  std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0).count());
    }
    tts_->start();

    // ── ROS ─────────────────────────────────────────────────────────────────
    speaking_pub_ = create_publisher<std_msgs::msg::Bool>("/is_speaking", 10);

    sub_say_ = create_subscription<std_msgs::msg::String>(
        in_topic, 20, [this](std_msgs::msg::String::ConstSharedPtr m) { tts_->say(m->data); });
    sub_int_ = create_subscription<std_msgs::msg::Empty>(
        "/interrupt_tts", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) { tts_->interrupt(); });
    sub_stop_ = create_subscription<std_msgs::msg::String>(
        "/stop_speaking", 10, [this](std_msgs::msg::String::ConstSharedPtr) { tts_->interrupt(); });

    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
      for (const auto& p : ps) {
        if (p.get_name() == "volume_gain") tts_->set_volume_gain(static_cast<float>(p.as_double()));
        else if (p.get_name() == "speed") tts_->set_speed(static_cast<float>(p.as_double()));
        else if (p.get_name() == "sid") tts_->set_sid(static_cast<int>(p.as_int()));
        else if (p.get_name() == "alsa_volume_percent") {
          alsa_pct_ = static_cast<int>(p.as_int());
          apply_alsa_volume();
        }
      }
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      return r;
    });

    // État parole : 50 Hz, publication sur changement + battement 1 Hz.
    state_timer_ = create_wall_timer(20ms, [this] { publish_state(); });
    // Chien de garde carte son.
    watchdog_ = create_wall_timer(1s, [this] { check_audio(); });

    if (!startup.empty()) tts_->say(startup);
    RCLCPP_INFO(get_logger(), "talk_node prêt — écoute %s", in_topic.c_str());
  }

  ~TalkNode() override {
    if (tts_) tts_->stop();
    if (out_) out_->close(true);
  }

 private:
  void apply_alsa_volume() {
    if (alsa_pct_ < 0) return;
    lat::set_alsa_card_volume(alsa_card_, alsa_pct_, /*playback=*/true,
                              lat::ros_logger(get_logger()));
  }

  void publish_state() {
    const bool s = tts_->busy();
    const auto now = std::chrono::steady_clock::now();
    if (s != last_state_ || now - last_pub_ > 1s) {
      std_msgs::msg::Bool m;
      m.data = s;
      speaking_pub_->publish(m);
      if (s != last_state_) RCLCPP_DEBUG(get_logger(), "is_speaking=%d", s);
      last_state_ = s;
      last_pub_ = now;
    }
  }

  void check_audio() {
    if (out_->healthy()) {
      fail_count_ = 0;
      return;
    }
    if (++fail_count_ % 3 != 1) return;  // un essai toutes les 3 s
    RCLCPP_WARN(get_logger(), "Carte son perdue — tentative de reconnexion…");
    tts_->interrupt();
    if (out_->reopen()) {
      apply_alsa_volume();
      RCLCPP_INFO(get_logger(), "Carte son reconnectée");
    }
  }

  std::unique_ptr<lat::AudioOutput> out_;
  std::unique_ptr<lat::TalkEngine> tts_;
  std::string alsa_card_;
  int alsa_pct_ = 100;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr speaking_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_say_, sub_stop_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_int_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr state_timer_, watchdog_;
  bool last_state_ = false;
  std::chrono::steady_clock::time_point last_pub_{};
  int fail_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<TalkNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("talk_node"), "%s", e.what());
    rc = 1;  // le launch (respawn) relancera le node
  }
  rclcpp::shutdown();
  return rc;
}
