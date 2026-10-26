/**
 * @file    qbo_tts_node.cpp
 * @brief   Node ROS2 TTS pour le robot Néo (plateforme QBo)
 *          v3 — stéréo 44100Hz natif + upsample ×2 + gain volume
 *
 * Le modèle sherpa-onnx génère du mono 22050Hz.
 * Le USB Audio Device (C-Media) exige du stéréo 44100Hz.
 * Solution : upsample ×2 (chaque sample répété) + duplication L/R.
 * Ratio entier → zéro distorsion, zéro dépendance externe.
 *
 * Paramètres ROS2 :
 *   model_path      : chemin vers axel.onnx
 *   tokens_path     : chemin vers tokens.txt
 *   espeak_data_dir : chemin vers espeak-ng-data
 *   num_threads     : threads ONNX (défaut 3)
 *   provider        : cuda / cpu (défaut cuda)
 *   sid             : speaker id (défaut 0)
 *   speed           : vitesse de parole (défaut 1.0)
 *   volume_gain     : amplification (défaut 2.0, modifiable à chaud)
 *   audio_device    : sous-chaîne du nom PortAudio (défaut "USB Audio Device")
 *
 * @license MIT
 * @author  Vincent Foucault — Avril 2026
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <portaudio.h>
#include "sherpa-onnx/c-api/c-api.h"

#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <string>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <vector>

// ─── Constantes audio ─────────────────────────────────────────────────────────
static constexpr int    MODEL_SAMPLE_RATE  = 22050; // sortie sherpa-onnx
static constexpr int    DEVICE_SAMPLE_RATE = 44100; // exigé par USB Audio Device
static constexpr int    UPSAMPLE_RATIO     = DEVICE_SAMPLE_RATE / MODEL_SAMPLE_RATE; // = 2
static constexpr int    DEVICE_CHANNELS    = 2;     // stéréo obligatoire
static constexpr size_t CHUNK_FRAMES_IN    = 512;   // frames mono 22050Hz
// frames stéréo 44100Hz correspondantes : ×2 (upsample) ×2 (stéréo) = ×4
static constexpr size_t CHUNK_FRAMES_OUT   = CHUNK_FRAMES_IN * UPSAMPLE_RATIO * DEVICE_CHANNELS;

// ─── Requête texte ────────────────────────────────────────────────────────────
struct TextRequest {
  std::string                           text;
  int32_t                               sid;
  float                                 speed;
  std::chrono::steady_clock::time_point received_at;
};

// ─── File d'attente thread-safe ───────────────────────────────────────────────
struct TextQueue {
  std::queue<TextRequest> queue;
  std::mutex              mutex;
  std::condition_variable cv;
};

static TextQueue         g_text_queue;
static std::atomic<bool> g_killed{false};
static std::atomic<bool> g_interrupt_flag{false};

static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr g_speaking_publisher = nullptr;

// ─── Gain volume (atomic float via memcpy/uint32) ────────────────────────────
static std::atomic<uint32_t> g_volume_gain_bits{0};

static float get_volume_gain() {
  uint32_t bits = g_volume_gain_bits.load(std::memory_order_relaxed);
  float val;
  std::memcpy(&val, &bits, sizeof(float));
  return val;
}

static void set_volume_gain(float v) {
  uint32_t bits;
  std::memcpy(&bits, &v, sizeof(float));
  g_volume_gain_bits.store(bits, std::memory_order_relaxed);
}

// ─── Sélection device PortAudio par nom partiel ───────────────────────────────
static PaDeviceIndex find_device_by_name(const std::string& hint) {
  int n = Pa_GetDeviceCount();
  if (hint.empty()) return Pa_GetDefaultOutputDevice();

  for (int i = 0; i < n; ++i) {
    const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
    if (!info || info->maxOutputChannels < 1) continue;
    if (std::string(info->name).find(hint) != std::string::npos) {
      RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
                  "Device audio sélectionné : [%d] %s  ch=%d  sr=%.0f",
                  i, info->name,
                  info->maxOutputChannels,
                  info->defaultSampleRate);
      return static_cast<PaDeviceIndex>(i);
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("qbo_tts"),
              "Device '%s' introuvable — fallback sur le device par défaut.",
              hint.c_str());
  return Pa_GetDefaultOutputDevice();
}

// ─── WorkerThread ─────────────────────────────────────────────────────────────
static void WorkerThread(const SherpaOnnxOfflineTts* tts,
                         const std::string&           audio_device_hint)
{
  Pa_Initialize();

  PaDeviceIndex device = find_device_by_name(audio_device_hint);
  if (device == paNoDevice) {
    RCLCPP_ERROR(rclcpp::get_logger("qbo_tts"), "Aucun device audio trouvé !");
    Pa_Terminate();
    return;
  }

  // ── Ouverture stream stéréo 44100Hz ─────────────────────────────────────────
  PaStreamParameters pa_params{};
  pa_params.device           = device;
  pa_params.channelCount     = DEVICE_CHANNELS;
  pa_params.sampleFormat     = paFloat32;
  pa_params.suggestedLatency = Pa_GetDeviceInfo(device)->defaultLowOutputLatency;

  PaStream* pa_stream = nullptr;
  PaError pa_err = Pa_OpenStream(&pa_stream, nullptr, &pa_params,
                                  DEVICE_SAMPLE_RATE,
                                  static_cast<unsigned long>(CHUNK_FRAMES_IN * UPSAMPLE_RATIO),
                                  paClipOff, nullptr, nullptr);
  if (pa_err != paNoError) {
    RCLCPP_ERROR(rclcpp::get_logger("qbo_tts"),
                 "Pa_OpenStream failed: %s", Pa_GetErrorText(pa_err));
    Pa_Terminate();
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
              "WorkerThread démarré — device: %s | %d Hz stéréo | upsample ×%d",
              Pa_GetDeviceInfo(device)->name,
              DEVICE_SAMPLE_RATE, UPSAMPLE_RATIO);

  // Buffer de sortie pré-alloué : stéréo 44100Hz
  std::vector<float> out_buf(CHUNK_FRAMES_OUT);

  while (!g_killed) {

    // ── Attente d'une requête ──────────────────────────────────────────────────
    TextRequest req;
    {
      std::unique_lock<std::mutex> lock(g_text_queue.mutex);
      g_text_queue.cv.wait(lock, [] {
        return !g_text_queue.queue.empty() || g_killed.load();
      });
      if (g_killed) break;
      req = std::move(g_text_queue.queue.front());
      g_text_queue.queue.pop();
    }

    // ── Inférence GPU sherpa-onnx ─────────────────────────────────────────────
    auto t0 = std::chrono::steady_clock::now();

    SherpaOnnxGenerationConfig gen_cfg{};
    gen_cfg.sid             = req.sid;
    gen_cfg.speed           = req.speed;
    gen_cfg.silence_scale   = 0.2f;
    gen_cfg.reference_audio = nullptr;

    const SherpaOnnxGeneratedAudio* audio =
      SherpaOnnxOfflineTtsGenerateWithConfig(
        tts, req.text.c_str(), &gen_cfg, nullptr, nullptr);

    auto t1 = std::chrono::steady_clock::now();

    if (!audio || audio->n == 0) {
      RCLCPP_WARN(rclcpp::get_logger("qbo_tts"),
                  "Audio vide pour : \"%s\"", req.text.c_str());
      if (audio) SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
      continue;
    }

    double ms_infer  = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double audio_ms  = (static_cast<double>(audio->n) / MODEL_SAMPLE_RATE) * 1000.0;
    RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
                "Inférence: %.0f ms | Audio: %.0f ms | RTF: %.3f",
                ms_infer, audio_ms, ms_infer / audio_ms);

    // ── Démarrage lecture — AVANT publication is_speaking ─────────────────────
    Pa_StartStream(pa_stream);

    if (g_speaking_publisher) {
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      g_speaking_publisher->publish(msg);
      RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
                  "🗣️  TTS : \"%s\"", req.text.c_str());
    }

    // ── Lecture par chunks avec upsample ×2 + stéréo + gain ──────────────────
    bool interrupted  = false;
    int  total        = audio->n;

    for (int offset = 0; offset < total; offset += static_cast<int>(CHUNK_FRAMES_IN)) {

      if (g_interrupt_flag.load()) {
        interrupted = true;
        Pa_AbortStream(pa_stream);
        RCLCPP_INFO(rclcpp::get_logger("qbo_tts"), "🛑 TTS interrompu");
        g_interrupt_flag.store(false);
        break;
      }

      // Nombre de samples mono source dans ce chunk
      int src_frames = std::min(static_cast<int>(CHUNK_FRAMES_IN), total - offset);
      float gain     = get_volume_gain();

      // Upsample ×2 + duplication stéréo L/R
      // Pour chaque sample mono src[i] :
      //   → on écrit UPSAMPLE_RATIO fois le sample en L et R
      //   → soit 2 × UPSAMPLE_RATIO floats par sample source
      int out_idx = 0;
      for (int i = 0; i < src_frames; ++i) {
        float s = std::clamp(audio->samples[offset + i] * gain, -1.0f, 1.0f);
        for (int r = 0; r < UPSAMPLE_RATIO; ++r) {
          out_buf[out_idx++] = s; // canal gauche
          out_buf[out_idx++] = s; // canal droit
        }
      }

      // Nombre de frames stéréo 44100Hz à écrire
      unsigned long out_frames = static_cast<unsigned long>(src_frames * UPSAMPLE_RATIO);

      PaError err = Pa_WriteStream(pa_stream, out_buf.data(), out_frames);
      if (err != paNoError && err != paOutputUnderflowed) {
        RCLCPP_WARN(rclcpp::get_logger("qbo_tts"),
                    "Pa_WriteStream: %s", Pa_GetErrorText(err));
      }
    }

    if (!interrupted) Pa_StopStream(pa_stream);

    // ── Publication fin de parole ─────────────────────────────────────────────
    if (g_speaking_publisher) {
      auto msg = std_msgs::msg::Bool();
      msg.data = false;
      g_speaking_publisher->publish(msg);
      if (!interrupted)
        RCLCPP_INFO(rclcpp::get_logger("qbo_tts"), "🔇 TTS fin de parole");
    }

    SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);

  } // fin while

  Pa_CloseStream(pa_stream);
  Pa_Terminate();
}

// ─── QboTTSNode ───────────────────────────────────────────────────────────────
class QboTTSNode : public rclcpp::Node {
public:
  QboTTSNode() : Node("qbo_tts_node") {

    // ── Déclaration des paramètres ────────────────────────────────────────────
    this->declare_parameter<std::string>("model_path",
      "install/qbo_tts/share/qbo_tts/models/axel.onnx");
    this->declare_parameter<std::string>("tokens_path",
      "install/qbo_tts/share/qbo_tts/models/tokens.txt");
    this->declare_parameter<std::string>("espeak_data_dir",
      "install/qbo_tts/share/qbo_tts/models/espeak-ng-data");
    this->declare_parameter<int>("num_threads", 3);
    this->declare_parameter<std::string>("provider", "cuda");
    this->declare_parameter<int>("sid", 0);
    this->declare_parameter<double>("speed", 1.0);
    this->declare_parameter<double>("volume_gain", 2.0);
    this->declare_parameter<std::string>("audio_device", "USB Audio Device");

    // ── Lecture des paramètres ────────────────────────────────────────────────
    std::string model_path   = this->get_parameter("model_path").as_string();
    std::string tokens_path  = this->get_parameter("tokens_path").as_string();
    std::string espeak_dir   = this->get_parameter("espeak_data_dir").as_string();
    int         num_threads  = this->get_parameter("num_threads").as_int();
    std::string provider     = this->get_parameter("provider").as_string();
    std::string audio_device = this->get_parameter("audio_device").as_string();
    float       gain         = static_cast<float>(
                                 this->get_parameter("volume_gain").as_double());

    set_volume_gain(gain);
    RCLCPP_INFO(get_logger(),
                "volume_gain=%.2f | audio_device=\"%s\"", gain, audio_device.c_str());

    // ── Callback modification gain à chaud ───────────────────────────────────
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
        -> rcl_interfaces::msg::SetParametersResult {
          for (const auto& p : params) {
            if (p.get_name() == "volume_gain") {
              float g = static_cast<float>(p.as_double());
              set_volume_gain(g);
              RCLCPP_INFO(get_logger(), "volume_gain → %.2f", g);
            }
          }
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          return res;
      });

    // ── Initialisation sherpa-onnx ────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Chargement TTS : %s", model_path.c_str());

    SherpaOnnxOfflineTtsConfig config{};
    config.model.vits.model    = model_path.c_str();
    config.model.vits.tokens   = tokens_path.c_str();
    config.model.vits.data_dir = espeak_dir.c_str();
    config.model.num_threads   = num_threads;
    config.model.provider      = provider.c_str();
    config.max_num_sentences   = 2;

    tts_ = SherpaOnnxCreateOfflineTts(&config);
    if (!tts_) {
      RCLCPP_ERROR(get_logger(), "Échec chargement modèle TTS !");
      rclcpp::shutdown();
      return;
    }

    int sr = SherpaOnnxOfflineTtsSampleRate(tts_);
    RCLCPP_INFO(get_logger(),
                "TTS prêt — modèle: %d Hz → device: %d Hz stéréo | provider: %s",
                sr, DEVICE_SAMPLE_RATE, provider.c_str());

    // ── Démarrage worker ──────────────────────────────────────────────────────
    worker_thread_ = std::thread(WorkerThread, tts_, audio_device);

    // ── Topics ────────────────────────────────────────────────────────────────
    speaking_pub_    = this->create_publisher<std_msgs::msg::Bool>("/is_speaking", 10);
    g_speaking_publisher = speaking_pub_;

    interrupt_sub_   = this->create_subscription<std_msgs::msg::Empty>(
      "/interrupt_tts", 10,
      [](std_msgs::msg::Empty::SharedPtr) { g_interrupt_flag.store(true); });

    subscription_    = this->create_subscription<std_msgs::msg::String>(
      "/to_speak", 10,
      std::bind(&QboTTSNode::speak_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "qbo_tts_node v3 prêt");
  }

  ~QboTTSNode() {
    g_killed = true;
    g_text_queue.cv.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
    if (tts_) SherpaOnnxDestroyOfflineTts(tts_);
  }

private:
  void speak_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;
    int   sid   = this->get_parameter("sid").as_int();
    float speed = static_cast<float>(this->get_parameter("speed").as_double());
    {
      std::lock_guard<std::mutex> lock(g_text_queue.mutex);
      g_text_queue.queue.push({msg->data, sid, speed,
                               std::chrono::steady_clock::now()});
    }
    g_text_queue.cv.notify_one();
  }

  const SherpaOnnxOfflineTts* tts_ = nullptr;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      speaking_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  interrupt_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::thread worker_thread_;
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QboTTSNode>());
  rclcpp::shutdown();
  return 0;
}