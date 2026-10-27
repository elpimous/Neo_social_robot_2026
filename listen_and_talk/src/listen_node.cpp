/**
 * @file    listen_node.cpp
 * @brief   Node ROS2 "listen" — reconnaissance vocale de Néo.
 *
 * Pipeline : ReSpeaker Lite 16 kHz → VAD (Silero / TEN) → Parakeet TDT
 * 0.6B v3 (sherpa-onnx, ponctuation + majuscules natives) → filtres.
 *
 * Topics :
 *   pub  /listened        std_msgs/String        texte reconnu (compatible)
 *   pub  /listen_result   qbo_msgs/ListenResult  texte + confiance
 *   pub  /interrupt_tts   std_msgs/Empty         barge-in détecté
 *   sub  /is_speaking     std_msgs/Bool          état parole du robot (écho)
 *   sub  /is_listening    std_msgs/Bool          active / coupe l'écoute
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <qbo_msgs/msg/listen_result.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "listen_and_talk/alsa_mixer.hpp"
#include "listen_and_talk/audio_input.hpp"
#include "listen_and_talk/listen_engine.hpp"
#include "ros_common.hpp"

using namespace std::chrono_literals;

class ListenNode : public rclcpp::Node {
 public:
  ListenNode() : Node("listen_node") {
    const std::string M = "pkg://listen_and_talk/models/listen/";
    // ── Modèles ─────────────────────────────────────────────────────────────
    lat::AsrEngine::Config ac;
    ac.model_type = declare_parameter<std::string>("asr_model_type", "nemo_transducer");
    const auto P = M + "sherpa-onnx-nemo-parakeet-tdt-0.6b-v3-int8/";
    ac.encoder = lat::resolve_path(declare_parameter<std::string>("asr_encoder", P + "encoder.int8.onnx"));
    ac.decoder = lat::resolve_path(declare_parameter<std::string>("asr_decoder", P + "decoder.int8.onnx"));
    ac.joiner = lat::resolve_path(declare_parameter<std::string>("asr_joiner", P + "joiner.int8.onnx"));
    ac.ctc_model = lat::resolve_path(declare_parameter<std::string>("asr_ctc_model", ""));
    ac.tokens = lat::resolve_path(declare_parameter<std::string>("asr_tokens", P + "tokens.txt"));
    ac.provider = declare_parameter<std::string>("asr_provider", "cpu");
    ac.num_threads = declare_parameter<int>("asr_num_threads", 4);

    lat::ListenPipeline::Config pc;
    pc.vad_model = lat::resolve_path(declare_parameter<std::string>("vad_model", M + "silero_vad.onnx"));
    pc.vad_type = declare_parameter<std::string>("vad_type", "auto");
    pc.vad_threshold = static_cast<float>(declare_parameter<double>("vad_threshold", 0.5));
    pc.min_silence_s = static_cast<float>(declare_parameter<double>("vad_min_silence_s", 0.6));
    pc.min_speech_s = static_cast<float>(declare_parameter<double>("vad_min_speech_s", 0.2));
    pc.max_speech_s = static_cast<float>(declare_parameter<double>("vad_max_speech_s", 12.0));
    pc.preroll_ms = declare_parameter<int>("preroll_ms", 250);
    pc.postroll_ms = declare_parameter<int>("postroll_ms", 150);
    pc.min_segment_s = static_cast<float>(declare_parameter<double>("min_segment_s", 0.35));
    pc.min_segment_rms = static_cast<float>(declare_parameter<double>("min_segment_rms", 0.004));
    pc.barge_in = declare_parameter<bool>("barge_in", true);
    pc.echo_tail_ms = declare_parameter<int>("echo_tail_ms", 450);
    pc.echo_latency_ms = declare_parameter<int>("echo_latency_ms", 120);
    pc.barge_in_grace_ms = declare_parameter<int>("barge_in_grace_ms", 350);
    pc.barge_in_ms = declare_parameter<int>("barge_in_ms", 160);
    pc.barge_in_window_ms = declare_parameter<int>("barge_in_window_ms", 640);
    pc.barge_in_ratio = static_cast<float>(declare_parameter<double>("barge_in_ratio", 2.0));
    pc.barge_in_min_rms = static_cast<float>(declare_parameter<double>("barge_in_min_rms", 0.02));

    lat::TextFilter tf;
    tf.min_chars = static_cast<size_t>(declare_parameter<int>("min_chars", 2));
    tf.ignore_words = declare_parameter<std::vector<std::string>>("ignore_words", tf.ignore_words);
    min_conf_ = declare_parameter<double>("min_confidence", 0.0);

    // ── Micro ───────────────────────────────────────────────────────────────
    lat::AudioInput::Config ic;
    ic.device_hint = declare_parameter<std::string>("mic_device", "Lite");
    ic.channels = declare_parameter<int>("mic_channels", 1);
    ic.channel_index = declare_parameter<int>("mic_channel_index", 0);
    mic_card_ = declare_parameter<std::string>("mic_alsa_card", "");
    mic_pct_ = declare_parameter<int>("mic_capture_percent", -1);
    if (mic_card_.empty()) mic_card_ = ic.device_hint;

    for (const auto& f : {pc.vad_model, ac.tokens})
      if (!lat::file_exists(f)) RCLCPP_ERROR(get_logger(), "Fichier introuvable : %s", f.c_str());

    auto log = lat::ros_logger(get_logger());
    apply_mic_gain();

    // ── ROS ─────────────────────────────────────────────────────────────────
    pub_text_ = create_publisher<std_msgs::msg::String>("/listened", 10);
    pub_res_ = create_publisher<qbo_msgs::msg::ListenResult>("/listen_result", 10);
    pub_int_ = create_publisher<std_msgs::msg::Empty>("/interrupt_tts", 10);

    in_ = std::make_unique<lat::AudioInput>(ic, log);
    engine_ = std::make_unique<lat::ListenEngine>(
        in_.get(), pc, ac, tf, log,
        [this](const lat::ListenEngine::Output& o) { on_text(o); },
        [this] { pub_int_->publish(std_msgs::msg::Empty()); });
    if (!engine_->init()) throw std::runtime_error("Échec chargement VAD/ASR");

    sub_spk_ = create_subscription<std_msgs::msg::Bool>(
        "/is_speaking", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          last_spk_msg_ = std::chrono::steady_clock::now();
          spk_state_ = m->data;
          engine_->pipeline().set_speaking(m->data);
        });
    sub_lis_ = create_subscription<std_msgs::msg::Bool>(
        "/is_listening", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data != engine_->pipeline().enabled())
            RCLCPP_INFO(get_logger(), m->data ? "🎙️  Écoute activée" : "🔇 Écoute coupée");
          engine_->pipeline().set_enabled(m->data);
        });

    if (!in_->open()) RCLCPP_ERROR(get_logger(), "Micro indisponible — nouvel essai automatique");
    engine_->start();
    watchdog_ = create_wall_timer(1s, [this] { check_mic(); });
    RCLCPP_INFO(get_logger(), "listen_node prêt — en écoute");
  }

  ~ListenNode() override {
    if (engine_) engine_->stop();
    if (in_) in_->close(true);
  }

 private:
  void apply_mic_gain() {
    if (mic_pct_ < 0) return;
    lat::set_alsa_card_volume(mic_card_, mic_pct_, /*playback=*/false,
                              lat::ros_logger(get_logger()));
  }

  void on_text(const lat::ListenEngine::Output& o) {
    if (min_conf_ > 0.0 && o.confidence >= 0.0f && o.confidence < min_conf_) {
      RCLCPP_INFO(get_logger(), "Confiance trop faible (%.2f) : [%s]", o.confidence, o.text.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "💬 [%.2fs | %.0f ms | conf %.2f%s] → %s", o.duration_s,
                o.compute_ms, o.confidence, o.after_barge_in ? " | barge-in" : "", o.text.c_str());
    std_msgs::msg::String s;
    s.data = o.text;
    pub_text_->publish(s);
    qbo_msgs::msg::ListenResult r;
    r.sentence = o.text;
    r.confidence = o.confidence;
    pub_res_->publish(r);
  }

  void check_mic() {
    // Sécurité : si talk_node meurt en pleine phrase, /is_speaking resterait
    // "vrai" pour toujours et toute parole serait jetée comme écho.
    if (spk_state_ && std::chrono::steady_clock::now() - last_spk_msg_ > 2500ms) {
      RCLCPP_WARN(get_logger(), "/is_speaking muet depuis 2.5 s — supposé faux");
      spk_state_ = false;
      engine_->pipeline().set_speaking(false);
    }
    if (in_->healthy()) {
      fail_count_ = 0;
      return;
    }
    if (++fail_count_ % 3 != 1) return;
    RCLCPP_WARN(get_logger(), "Micro perdu — tentative de reconnexion…");
    if (in_->reopen()) {
      apply_mic_gain();
      RCLCPP_INFO(get_logger(), "Micro reconnecté");
    }
  }

  std::unique_ptr<lat::AudioInput> in_;
  std::unique_ptr<lat::ListenEngine> engine_;
  std::string mic_card_;
  int mic_pct_ = -1;
  double min_conf_ = 0.0;
  int fail_count_ = 0;
  bool spk_state_ = false;
  std::chrono::steady_clock::time_point last_spk_msg_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_text_;
  rclcpp::Publisher<qbo_msgs::msg::ListenResult>::SharedPtr pub_res_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_int_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_spk_, sub_lis_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<ListenNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("listen_node"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
