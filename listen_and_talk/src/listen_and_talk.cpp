/**
 * @file listen_and_talk_node.cpp
 * @brief Node ROS2 unifié STT + TTS — Robot Néo
 *
 * ══════════════════════════════════════════════════════════════════
 *  Pipeline STT (TensorRT, GPU) :
 *    ReSpeaker Lite 16kHz mono
 *      → RNNoise (débruitage neural, librnnoise.so CPU)
 *      → RMS hysteresis (anti-hallucination, seuils ON/OFF)
 *      → accumulation parole → mel spectrogram (NeMo-compatible)
 *      → Parakeet TDT 0.6B encoder (TRT FP16)
 *      → TDT greedy decoder  (TRT FP16)
 *      → publication /listened
 *
 *  Pipeline TTS (sherpa-onnx VITS, CUDA) :
 *    /to_speak → VITS → resample 48kHz stéréo S16 → USB Audio Device
 *
 *  Interruption :
 *    Micro détecte parole → g_interrupt coupe TTS mid-phrase.
 * ══════════════════════════════════════════════════════════════════
 *  Author : Vincent Foucault — 2026
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <portaudio.h>
#include <yaml-cpp/yaml.h>

// TensorRT + CUDA
#include <cuda_fp16.h>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

// Sherpa-ONNX (TTS uniquement)
#include "sherpa-onnx/c-api/c-api.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ═══════════════════════════════════════════════════════════════════════════════
//  Constantes globales
// ═══════════════════════════════════════════════════════════════════════════════
static constexpr int32_t kOutputSampleRate = 48000;  // USB Audio Device
static constexpr int32_t kSTTSampleRate    = 16000;
static constexpr int     kChunkSize        = 512;

// Mel spectrogram — NeMo Parakeet
static constexpr int kNFFT      = 512;
static constexpr int kHop       = 160;
static constexpr int kWinLen    = 320;
static constexpr int kNMels     = 128;
static constexpr int kNBins     = kNFFT / 2 + 1;  // 257
static constexpr float kFMin    = 0.0f;
static constexpr float kFMax    = 8000.0f;

// TDT decoder
static constexpr int kDurations[] = {0, 1, 2, 3, 4};
static constexpr int kNDur        = 5;

// ═══════════════════════════════════════════════════════════════════════════════
//  Variables globales (synchronisation TTS / STT)
// ═══════════════════════════════════════════════════════════════════════════════
struct TextRequest {
    std::string text;
    int32_t sid;
    float speed;
};

struct TextQueue {
    std::queue<TextRequest> queue;
    std::mutex mutex;
    std::condition_variable cv;
};

static TextQueue g_text_queue;
static std::atomic<bool> g_killed{false};
static std::atomic<bool> g_interrupt{false};

// ═══════════════════════════════════════════════════════════════════════════════
//  RNNoise — débruitage neural via librnnoise.so
// ═══════════════════════════════════════════════════════════════════════════════
extern "C" {
    void*  rnnoise_create(void* model);
    void   rnnoise_destroy(void* st);
    float  rnnoise_process_frame(void* st, float* out, const float* in);
}

class RNNoise {
public:
    static constexpr int FRAME_48K = 480;
    static constexpr int UP = 3;

    RNNoise() : st_(rnnoise_create(nullptr)) {}
    ~RNNoise() { if (st_) rnnoise_destroy(st_); }

    // vince : débruite chunk_16k in-place (512 samples float32 [-1,1])
    void process(float* chunk_16k, int n) {
        // 16kHz → 48kHz (repeat)
        std::vector<float> up(n * UP);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < UP; ++j)
                up[i * UP + j] = chunk_16k[i];

        // Concatène overlap
        std::vector<float> buf;
        buf.reserve(overlap_.size() + up.size());
        buf.insert(buf.end(), overlap_.begin(), overlap_.end());
        buf.insert(buf.end(), up.begin(), up.end());

        // Traitement frames de 480
        std::vector<float> denoised;
        denoised.reserve(buf.size());
        int pos = 0;
        float fin[FRAME_48K], fout[FRAME_48K];

        while (pos + FRAME_48K <= static_cast<int>(buf.size())) {
            for (int i = 0; i < FRAME_48K; ++i)
                fin[i] = buf[pos + i] * 32768.0f;
            rnnoise_process_frame(st_, fout, fin);
            for (int i = 0; i < FRAME_48K; ++i)
                denoised.push_back(fout[i] / 32768.0f);
            pos += FRAME_48K;
        }

        overlap_.assign(buf.begin() + pos, buf.end());
        if (denoised.empty()) return;

        // 48kHz → 16kHz (moyenne triplets)
        int trim = (static_cast<int>(denoised.size()) / UP) * UP;
        int ds_n = trim / UP;
        for (int i = 0; i < n; ++i) {
            if (i < ds_n) {
                int b = i * UP;
                chunk_16k[i] = (denoised[b] + denoised[b+1] + denoised[b+2]) / 3.0f;
            } else {
                chunk_16k[i] = 0.0f;
            }
        }
    }

private:
    void* st_;
    std::vector<float> overlap_;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  FFT radix-2 in-place (N=512)
// ═══════════════════════════════════════════════════════════════════════════════
static void fft_inplace(float* re, float* im, int N) {
    // Bit reversal
    for (int i = 1, j = 0; i < N; ++i) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { std::swap(re[i], re[j]); std::swap(im[i], im[j]); }
    }
    // Butterflies
    for (int len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * static_cast<float>(M_PI) / len;
        float wre = cosf(ang), wim = sinf(ang);
        for (int i = 0; i < N; i += len) {
            float cre = 1.0f, cim = 0.0f;
            for (int j = 0; j < len / 2; ++j) {
                int a = i + j, b = i + j + len / 2;
                float tre = re[b] * cre - im[b] * cim;
                float tim = re[b] * cim + im[b] * cre;
                re[b] = re[a] - tre;  im[b] = im[a] - tim;
                re[a] += tre;         im[a] += tim;
                float nre = cre * wre - cim * wim;
                cim = cre * wim + cim * wre;
                cre = nre;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MelExtractor — mel spectrogram NeMo-compatible
//  Identique au Python : librosa params, center=True, reflect padding,
//  Hann window, log(mel + 2^-24), normalize per-bin
// ═══════════════════════════════════════════════════════════════════════════════
class MelExtractor {
public:
    MelExtractor() {
        // Hann window
        for (int i = 0; i < kWinLen; ++i)
            hann_[i] = 0.5f * (1.0f - cosf(2.0f * static_cast<float>(M_PI) * i / kWinLen));

        // Mel filterbank triangulaire (HTK scale)
        float mel_min = hz2mel(kFMin), mel_max = hz2mel(kFMax);
        float mels[kNMels + 2], hz[kNMels + 2];
        for (int i = 0; i < kNMels + 2; ++i) {
            mels[i] = mel_min + i * (mel_max - mel_min) / (kNMels + 1);
            hz[i] = mel2hz(mels[i]);
        }

        float fft_freqs[kNBins];
        for (int i = 0; i < kNBins; ++i)
            fft_freqs[i] = static_cast<float>(i) * kSTTSampleRate / kNFFT;

        std::memset(mel_basis_, 0, sizeof(mel_basis_));
        for (int m = 0; m < kNMels; ++m) {
            for (int k = 0; k < kNBins; ++k) {
                float f = fft_freqs[k];
                if (f >= hz[m] && f <= hz[m+1] && hz[m+1] > hz[m])
                    mel_basis_[m][k] = (f - hz[m]) / (hz[m+1] - hz[m]);
                else if (f > hz[m+1] && f <= hz[m+2] && hz[m+2] > hz[m+1])
                    mel_basis_[m][k] = (hz[m+2] - f) / (hz[m+2] - hz[m+1]);
            }
        }
    }

    // vince : extrait mel [1, 128, T] (row-major) + length
    void extract(const float* wav, int wav_n,
                 std::vector<float>& mel_out, int64_t& length_out) {

        // Reflect padding (center=True, comme librosa)
        int pad = kNFFT / 2;
        int padded_n = wav_n + 2 * pad;
        std::vector<float> padded(padded_n);
        for (int i = 0; i < pad; ++i)
            padded[i] = wav[std::min(pad - i, wav_n - 1)];
        std::memcpy(padded.data() + pad, wav, wav_n * sizeof(float));
        for (int i = 0; i < pad; ++i)
            padded[pad + wav_n + i] = wav[std::max(wav_n - 2 - i, 0)];

        int T = 1 + (padded_n - kNFFT) / kHop;

        // Power spectrogram via STFT
        std::vector<float> power(T * kNBins);
        float re[kNFFT], im[kNFFT];

        for (int t = 0; t < T; ++t) {
            int start = t * kHop;
            std::memset(re, 0, sizeof(re));
            std::memset(im, 0, sizeof(im));
            for (int i = 0; i < kWinLen && start + i < padded_n; ++i)
                re[i] = padded[start + i] * hann_[i];
            fft_inplace(re, im, kNFFT);
            for (int k = 0; k < kNBins; ++k)
                power[t * kNBins + k] = re[k] * re[k] + im[k] * im[k];
        }

        // Mel filterbank + log
        mel_out.resize(kNMels * T);
        for (int m = 0; m < kNMels; ++m) {
            for (int t = 0; t < T; ++t) {
                float sum = 0;
                for (int k = 0; k < kNBins; ++k)
                    sum += mel_basis_[m][k] * power[t * kNBins + k];
                mel_out[m * T + t] = logf(sum + 5.96e-8f);  // 2^-24
            }
        }

        // Normalize per mel bin (mean/std)
        for (int m = 0; m < kNMels; ++m) {
            float mean = 0;
            for (int t = 0; t < T; ++t) mean += mel_out[m * T + t];
            mean /= T;
            float var = 0;
            for (int t = 0; t < T; ++t) {
                float d = mel_out[m * T + t] - mean;
                var += d * d;
            }
            float s = sqrtf(var / T) + 1e-5f;
            for (int t = 0; t < T; ++t)
                mel_out[m * T + t] = (mel_out[m * T + t] - mean) / s;
        }

        // Padding min / clip max
        if (T < 20) {
            std::vector<float> padmel(kNMels * 20, 0.0f);
            for (int m = 0; m < kNMels; ++m)
                std::memcpy(&padmel[m * 20], &mel_out[m * T], T * sizeof(float));
            mel_out = std::move(padmel);
            T = 20;
        }
        if (T > 1000) {
            std::vector<float> clipped(kNMels * 1000);
            for (int m = 0; m < kNMels; ++m)
                std::memcpy(&clipped[m * 1000], &mel_out[m * T], 1000 * sizeof(float));
            mel_out = std::move(clipped);
            T = 1000;
        }
        length_out = T;
    }

private:
    float hann_[kWinLen];
    float mel_basis_[kNMels][kNBins];
    static float hz2mel(float f) { return 2595.0f * log10f(1.0f + f / 700.0f); }
    static float mel2hz(float m) { return 700.0f * (powf(10.0f, m / 2595.0f) - 1.0f); }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  TRTEngine — wrapper TensorRT C++ avec buffers GPU pré-alloués
//  Réallocation uniquement si la shape change (zéro malloc au runtime pour
//  le decoder dont les shapes sont fixes 1×1).
// ═══════════════════════════════════════════════════════════════════════════════
class TRTEngine {
    struct GpuBuf { void* ptr = nullptr; size_t cap = 0; };

    struct Logger : nvinfer1::ILogger {
        void log(Severity s, const char* m) noexcept override {
            if (s <= Severity::kWARNING)
                fprintf(stderr, "[TRT] %s\n", m);
        }
    };
    static Logger sLogger_;

    nvinfer1::ICudaEngine*       engine_  = nullptr;
    nvinfer1::IExecutionContext* ctx_     = nullptr;
    cudaStream_t                 stream_  = nullptr;
    nvinfer1::IRuntime*          runtime_ = nullptr;

    std::unordered_map<std::string, GpuBuf>               d_bufs_;
    std::unordered_map<std::string, std::vector<uint8_t>>  h_out_;

public:
    TRTEngine() = default;
    ~TRTEngine() { destroy(); }

    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f) return false;
        size_t sz = f.tellg();
        f.seekg(0);
        std::vector<char> data(sz);
        f.read(data.data(), sz);
        runtime_ = nvinfer1::createInferRuntime(sLogger_);
        engine_ = runtime_->deserializeCudaEngine(data.data(), sz);
        if (!engine_) return false;

        ctx_ = engine_->createExecutionContext();
        cudaStreamCreate(&stream_);
        return ctx_ != nullptr;
    }

    void destroy() {
        if (stream_) cudaStreamSynchronize(stream_);

        for (auto& [_, b] : d_bufs_)
            if (b.ptr) cudaFree(b.ptr);

        d_bufs_.clear();
        h_out_.clear();

        if (ctx_)    { delete ctx_;    ctx_ = nullptr; }
        if (engine_) { delete engine_; engine_ = nullptr; }
        if (runtime_){ delete runtime_; runtime_ = nullptr; }

        if (stream_) { cudaStreamDestroy(stream_); stream_ = nullptr; }
    }

    void setInput(const char* name, const void* host, nvinfer1::Dims dims) {
        ctx_->setInputShape(name, dims);
        size_t bytes = tensorBytes(name, dims);
        ensureBuf(name, bytes);
        cudaMemcpyAsync(d_bufs_[name].ptr, host, bytes,
                        cudaMemcpyHostToDevice, stream_);
        ctx_->setTensorAddress(name, d_bufs_[name].ptr);
    }

    bool execute() {
        int n = engine_->getNbIOTensors();
        for (int i = 0; i < n; ++i) {
            const char* name = engine_->getIOTensorName(i);
            if (engine_->getTensorIOMode(name) != nvinfer1::TensorIOMode::kOUTPUT)
                continue;
            auto dims = ctx_->getTensorShape(name);
            size_t bytes = tensorBytes(name, dims);
            ensureBuf(name, bytes);
            ctx_->setTensorAddress(name, d_bufs_[name].ptr);
            h_out_[name].resize(bytes);
        }
        if (!ctx_->enqueueV3(stream_)) return false;
        for (auto& [name, hb] : h_out_)
            cudaMemcpyAsync(hb.data(), d_bufs_[name].ptr, hb.size(),
                            cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);
        return true;
    }

    const void* output(const char* name) const {
        auto it = h_out_.find(name);
        return it != h_out_.end() ? it->second.data() : nullptr;
    }

    nvinfer1::Dims outputDims(const char* name) const {
        return ctx_->getTensorShape(name);
    }
    nvinfer1::DataType getDataType(const char* name) const {
        return engine_->getTensorDataType(name);
    }

private:
    void ensureBuf(const std::string& n, size_t bytes) {
        auto& b = d_bufs_[n];
        if (b.cap < bytes) {
            if (b.ptr) cudaFree(b.ptr);
            cudaMalloc(&b.ptr, bytes);
            b.cap = bytes;
        }
    }

    size_t tensorBytes(const char* name, nvinfer1::Dims dims) const {
        size_t es = 4;
        switch (engine_->getTensorDataType(name)) {
            case nvinfer1::DataType::kHALF:  es = 2; break;
            case nvinfer1::DataType::kINT64: es = 8; break;
            case nvinfer1::DataType::kINT8:  es = 1; break;
            case nvinfer1::DataType::kBOOL:  es = 1; break;
            default: es = 4; break;
        }
        size_t total = es;
        for (int i = 0; i < dims.nbDims; ++i) total *= dims.d[i];
        return total;
    }
};

TRTEngine::Logger TRTEngine::sLogger_;

// ═══════════════════════════════════════════════════════════════════════════════
//  ParakeetASR — encoder + greedy TDT decoder, 100% TRT
// ═══════════════════════════════════════════════════════════════════════════════
class ParakeetASR {
public:
    bool init(const std::string& enc_path, const std::string& dec_path,
              const std::string& vocab_path) {
        if (!encoder_.load(enc_path)) return false;
        if (!decoder_.load(dec_path)) return false;
        loadVocab(vocab_path);
        return true;
    }

    void destroy() { encoder_.destroy(); decoder_.destroy(); }

    std::string transcribe(const float* wav, int wav_n) {
        std::vector<float> mel;
        int64_t length;
        mel_.extract(wav, wav_n, mel, length);

        // Encoder
        nvinfer1::Dims mel_dims{};
        mel_dims.nbDims = 3;
        mel_dims.d[0] = 1; mel_dims.d[1] = kNMels; mel_dims.d[2] = length;

        nvinfer1::Dims len_dims{};
        len_dims.nbDims = 1; len_dims.d[0] = 1;

        encoder_.setInput("audio_signal", mel.data(), mel_dims);
        encoder_.setInput("length", &length, len_dims);
        if (!encoder_.execute()) return "";

        auto enc_odims = encoder_.outputDims("outputs");
        int D  = enc_odims.d[1];   // hidden dim
        int T  = enc_odims.d[2];   // downsampled time
        const auto* enc_raw = encoder_.output("outputs");
        if (!enc_raw) return "";

        std::vector<float> enc_out(D * T);

        auto dtype = encoder_.getDataType("outputs");

        if (dtype == nvinfer1::DataType::kHALF) {
            const __half* hptr = static_cast<const __half*>(enc_raw);
            for (int i = 0; i < D * T; ++i) {
                enc_out[i] = __half2float(hptr[i]);
            }
        } else {
            std::memcpy(enc_out.data(), enc_raw, D * T * sizeof(float));
        }

        auto tokens = greedyDecode(enc_out.data(), D, T);
        return tokensToText(tokens);
    }

private:
    TRTEngine   encoder_, decoder_;
    MelExtractor mel_;
    std::unordered_map<int, std::string> vocab_;
    int blank_id_   = 0;
    int vocab_size_ = 0;

    void loadVocab(const std::string& path) {
        vocab_.clear();
        std::ifstream f(path);
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            auto sp = line.find_last_of(" \t");
            if (sp == std::string::npos) continue;
            std::string tok = line.substr(0, sp);
            int id = std::stoi(line.substr(sp + 1));
            vocab_[id] = tok;
        }
        blank_id_ = 0;
        for (auto& [id, _] : vocab_)
            blank_id_ = std::max(blank_id_, id);
        vocab_size_ = blank_id_ + 1;
    }

    std::vector<int> greedyDecode(const float* enc, int D, int T) {
        // vince : états decoder fp16 — 2×1×640 = 1280 half = 2560 bytes
        static constexpr int kStateElems = 2 * 1 * 640;
        static constexpr int kStateBytes = kStateElems * 2;  // fp16

        std::vector<uint8_t> state1(kStateBytes, 0);
        std::vector<uint8_t> state2(kStateBytes, 0);
        int64_t target[1]  = { static_cast<int64_t>(blank_id_) };
        int64_t tgt_len[1] = { 1 };

        nvinfer1::Dims enc_s{};  enc_s.nbDims = 3;
        enc_s.d[0] = 1; enc_s.d[1] = D; enc_s.d[2] = 1;

        nvinfer1::Dims tgt_d{};  tgt_d.nbDims = 2;
        tgt_d.d[0] = 1; tgt_d.d[1] = 1;

        nvinfer1::Dims tl_d{};   tl_d.nbDims = 1;  tl_d.d[0] = 1;

        nvinfer1::Dims st_d{};   st_d.nbDims = 3;
        st_d.d[0] = 2; st_d.d[1] = 1; st_d.d[2] = 640;

        std::vector<float> enc_slice(D);
        std::vector<int> tokens;
        int t = 0;

        while (t < T) {
            // vince : slice enc[:, :, t] → [1, D, 1]
            for (int d = 0; d < D; ++d)
                enc_slice[d] = enc[d * T + t];

            decoder_.setInput("encoder_outputs", enc_slice.data(), enc_s);
            decoder_.setInput("targets",         target,           tgt_d);
            decoder_.setInput("target_length",   tgt_len,          tl_d);
            decoder_.setInput("input_states_1",  state1.data(),    st_d);
            decoder_.setInput("input_states_2",  state2.data(),    st_d);

            if (!decoder_.execute()) break;

            // Logits [1, 1, 1, vocab_size + 5]
            auto lo_dims = decoder_.outputDims("outputs");
            int logits_dim = lo_dims.d[lo_dims.nbDims - 1];
            const float* logits = static_cast<const float*>(decoder_.output("outputs"));

            // Copier nouveaux états
            std::memcpy(state1.data(), decoder_.output("output_states_1"), kStateBytes);
            std::memcpy(state2.data(), decoder_.output("output_states_2"), kStateBytes);

            // Token prediction (argmax sur [:vocab_size])
            int pred_tok = 0;
            float max_l  = logits[0];
            for (int i = 1; i < vocab_size_; ++i) {
                if (logits[i] > max_l) { max_l = logits[i]; pred_tok = i; }
            }

            // Duration prediction (argmax sur [vocab_size:])
            int pred_dur_idx = 0;
            float max_d = logits[vocab_size_];
            for (int i = 1; i < kNDur && vocab_size_ + i < logits_dim; ++i) {
                if (logits[vocab_size_ + i] > max_d) {
                    max_d = logits[vocab_size_ + i];
                    pred_dur_idx = i;
                }
            }
            int pred_dur = kDurations[pred_dur_idx];

            if (pred_tok == blank_id_) {
                t += std::max(pred_dur, 1);
            } else {
                tokens.push_back(pred_tok);
                target[0] = pred_tok;
                t += pred_dur;
            }
        }
        return tokens;
    }

    std::string tokensToText(const std::vector<int>& tokens) {
        std::string raw;
        for (int t : tokens) {
            auto it = vocab_.find(t);
            if (it != vocab_.end()) raw += it->second;
        }
        // Remplacer ▁ (UTF-8: E2 96 81) par espace
        std::string out;
        for (size_t i = 0; i < raw.size(); ) {
            if (i + 2 < raw.size() &&
                static_cast<uint8_t>(raw[i])   == 0xE2 &&
                static_cast<uint8_t>(raw[i+1]) == 0x96 &&
                static_cast<uint8_t>(raw[i+2]) == 0x81) {
                out += ' ';
                i += 3;
            } else {
                out += raw[i++];
            }
        }
        // Trim
        auto s = out.find_first_not_of(' ');
        auto e = out.find_last_not_of(' ');
        return (s == std::string::npos) ? "" : out.substr(s, e - s + 1);
    }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Utilitaires audio
// ═══════════════════════════════════════════════════════════════════════════════
PaDeviceIndex GetDeviceByName(const std::string& name, bool isInput) {
    int n = Pa_GetDeviceCount();
    for (int i = 0; i < n; ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (!info) continue;
        if (std::string(info->name).find(name) != std::string::npos) {
            if (isInput && info->maxInputChannels > 0) return i;
            if (!isInput && info->maxOutputChannels > 0) return i;
        }
    }
    return paNoDevice;
}

static float ComputeRMS(const float* buf, int n) {
    float s = 0;
    for (int i = 0; i < n; ++i) s += buf[i] * buf[i];
    return sqrtf(s / n);
}

static std::vector<int16_t> ResampleToStereoInt16(
    const float* src, int32_t src_n, int32_t src_rate, int32_t dst_rate)
{
    double ratio = static_cast<double>(dst_rate) / src_rate;
    int32_t dst_n = static_cast<int32_t>(std::ceil(src_n * ratio));
    std::vector<int16_t> out(dst_n * 2);
    for (int32_t i = 0; i < dst_n; ++i) {
        double pos = i / ratio;
        int32_t idx = static_cast<int32_t>(pos);
        float frac  = static_cast<float>(pos - idx);
        float sample = (idx + 1 < src_n)
            ? src[idx] * (1.0f - frac) + src[idx + 1] * frac
            : src[std::min(idx, src_n - 1)];
        sample = std::clamp(sample, -1.0f, 1.0f);
        int16_t s16 = static_cast<int16_t>(sample * 32767.0f);
        out[i * 2] = out[i * 2 + 1] = s16;
    }
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WorkerThread TTS — sherpa-onnx VITS + resample + interruption mid-phrase
// ═══════════════════════════════════════════════════════════════════════════════
void WorkerThread(const SherpaOnnxOfflineTts* tts, int32_t model_sr) {

    PaStreamParameters outP{};
    outP.device = GetDeviceByName("USB Audio Device", false);
    if (outP.device == paNoDevice) {
        RCLCPP_ERROR(rclcpp::get_logger("tts"), "Output device not found!");
        return;
    }
    outP.channelCount     = 2;
    outP.sampleFormat     = paInt16;
    outP.suggestedLatency = Pa_GetDeviceInfo(outP.device)->defaultLowOutputLatency;

    PaStream* pa = nullptr;
    if (Pa_OpenStream(&pa, nullptr, &outP, kOutputSampleRate, 512,
                      paClipOff, nullptr, nullptr) != paNoError) return;

    RCLCPP_INFO(rclcpp::get_logger("tts"),
                "TTS prêt (%d Hz → %d Hz stéréo)", model_sr, kOutputSampleRate);

    while (!g_killed) {
        TextRequest req;
        {
            std::unique_lock<std::mutex> lk(g_text_queue.mutex);
            g_text_queue.cv.wait(lk, [] {
                return !g_text_queue.queue.empty() || g_killed || g_interrupt;
            });
            if (g_killed) break;
            if (g_interrupt) {
                std::queue<TextRequest> e;
                std::swap(g_text_queue.queue, e);
                g_interrupt = false;
                continue;
            }
            req = std::move(g_text_queue.queue.front());
            g_text_queue.queue.pop();
        }

        SherpaOnnxGenerationConfig gc{};
        gc.sid = req.sid;  gc.speed = req.speed;
        gc.silence_scale = 0.2f;  gc.reference_audio = nullptr;

        const SherpaOnnxGeneratedAudio* audio =
            SherpaOnnxOfflineTtsGenerateWithConfig(tts, req.text.c_str(),
                                                   &gc, nullptr, nullptr);
        if (!audio || audio->n == 0) {
            if (audio) SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
            continue;
        }

        auto resampled = ResampleToStereoInt16(audio->samples, audio->n,
                                               model_sr, kOutputSampleRate);
        SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);

        // vince : écriture par blocs pour permettre l'interruption mid-phrase
        Pa_StartStream(pa);
        constexpr unsigned long kBlk = 1024;
        unsigned long total = static_cast<unsigned long>(resampled.size() / 2);
        unsigned long done  = 0;
        while (done < total && !g_interrupt && !g_killed) {
            unsigned long w = std::min(total - done, kBlk);
            Pa_WriteStream(pa, resampled.data() + done * 2, w);
            done += w;
        }
        Pa_StopStream(pa);

        if (g_interrupt) {
            std::lock_guard<std::mutex> lk(g_text_queue.mutex);
            std::queue<TextRequest> e;
            std::swap(g_text_queue.queue, e);
            g_interrupt = false;
        }
    }
    Pa_CloseStream(pa);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Node ListenAndTalk
// ═══════════════════════════════════════════════════════════════════════════════
class ListenAndTalkNode : public rclcpp::Node {
public:
    ListenAndTalkNode() : Node("listen_and_talk_node") {

        declare_parameter<int>("sid", 0);
        declare_parameter<double>("speed", 1.0);

        PaError pe = Pa_Initialize();
        if (pe != paNoError) {
            RCLCPP_ERROR(get_logger(), "Pa_Initialize: %s", Pa_GetErrorText(pe));
            rclcpp::shutdown(); return;
        }

        auto pkg = ament_index_cpp::get_package_share_directory("listen_and_talk");
        YAML::Node cfg = YAML::LoadFile(pkg + "/config/config_params.yaml");

        // ── Chemins TTS ─────────────────────────────────────────────────────
        tts_model_   = pkg + "/models/" + cfg["tts_vits_model"].as<std::string>();
        tts_tokens_  = pkg + "/models/" + cfg["tts_tokens_path"].as<std::string>();
        tts_espeak_  = pkg + "/models/" + cfg["tts_espeak_data_dir"].as<std::string>();

        // ── Chemins STT ─────────────────────────────────────────────────────
        stt_encoder_   = pkg + "/models/" + cfg["stt_encoder"].as<std::string>();
        stt_decoder_   = pkg + "/models/" + cfg["stt_decoder"].as<std::string>();
        stt_vocab_     = pkg + "/models/" + cfg["stt_vocab_file"].as<std::string>();

        // ── Init TTS ────────────────────────────────────────────────────────
        SherpaOnnxOfflineTtsConfig tc{};
        std::memset(&tc, 0, sizeof(tc));
        tc.model.vits.model    = tts_model_.c_str();
        tc.model.vits.tokens   = tts_tokens_.c_str();
        tc.model.vits.data_dir = tts_espeak_.c_str();
        tc.model.num_threads   = 3;
        tc.model.provider      = "cuda";
        tc.max_num_sentences   = 2;

        tts_ = SherpaOnnxCreateOfflineTts(&tc);
        if (!tts_) {
            RCLCPP_ERROR(get_logger(), "Erreur chargement TTS !");
            rclcpp::shutdown(); return;
        }
        tts_sr_ = SherpaOnnxOfflineTtsSampleRate(tts_);
        RCLCPP_INFO(get_logger(), "✅ TTS chargé (%d Hz)", tts_sr_);

        tts_thread_ = std::thread(WorkerThread, tts_, tts_sr_);

        // ── ROS2 topics ─────────────────────────────────────────────────────
        pub_ = create_publisher<std_msgs::msg::String>("/listened", 10);

        sub_speak_ = create_subscription<std_msgs::msg::String>(
            "/to_speak", 10, [this](const std_msgs::msg::String::SharedPtr m) {
                if (m->data.empty()) return;
                int sid = get_parameter("sid").as_int();
                float spd = static_cast<float>(get_parameter("speed").as_double());
                std::lock_guard<std::mutex> lk(g_text_queue.mutex);
                g_text_queue.queue.push({m->data, sid, spd});
                g_text_queue.cv.notify_one();
            });

        sub_stop_ = create_subscription<std_msgs::msg::String>(
            "/stop_speaking", 10, [](const std_msgs::msg::String::SharedPtr) {
                g_interrupt = true;
            });

        // ── Thread STT ──────────────────────────────────────────────────────
        stt_thread_ = std::thread(&ListenAndTalkNode::SttLoop, this);
    }

    ~ListenAndTalkNode() {
        g_killed = true;
        g_text_queue.cv.notify_all();
        if (tts_thread_.joinable()) tts_thread_.join();
        if (stt_thread_.joinable()) stt_thread_.join();
        if (tts_) SherpaOnnxDestroyOfflineTts(tts_);
        Pa_Terminate();
    }

private:
    const SherpaOnnxOfflineTts* tts_ = nullptr;
    int32_t tts_sr_ = 22050;

    std::string tts_model_, tts_tokens_, tts_espeak_;
    std::string stt_encoder_, stt_decoder_, stt_vocab_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_speak_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_stop_;

    std::thread tts_thread_, stt_thread_;

    // ═════════════════════════════════════════════════════════════════════════
    //  Boucle STT — TRT Parakeet TDT
    //
    //  Port du pipeline Python :
    //    PortAudio 16kHz mono → RNNoise → RMS hysteresis →
    //    accumulation buffer parole → mel + encoder + TDT decoder →
    //    publication /listened
    // ═════════════════════════════════════════════════════════════════════════
    void SttLoop() {

        cudaSetDevice(0);

        // ── Micro ReSpeaker Lite ────────────────────────────────────────────
        PaStreamParameters inP{};
        PaDeviceIndex inDev = GetDeviceByName("Lite", true);
        if (inDev == paNoDevice) {
            RCLCPP_ERROR(get_logger(), "Micro ReSpeaker non trouvé !");
            return;
        }
        inP.device           = inDev;
        inP.channelCount     = 1;
        inP.sampleFormat     = paInt16;
        inP.suggestedLatency = Pa_GetDeviceInfo(inDev)->defaultLowInputLatency;

        PaStream* mic = nullptr;
        PaError err = Pa_OpenStream(&mic, &inP, nullptr, kSTTSampleRate,
                                    kChunkSize, paClipOff, nullptr, nullptr);
        if (err != paNoError) {
            RCLCPP_ERROR(get_logger(), "STT Pa_OpenStream: %s", Pa_GetErrorText(err));
            return;
        }
        Pa_StartStream(mic);
        RCLCPP_INFO(get_logger(), "🎙️  Micro ouvert (device %d, 16kHz mono)", inDev);

        // ── Charger engines TRT ─────────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "⚡ Chargement engines TensorRT...");
        ParakeetASR asr;
        if (!asr.init(stt_encoder_, stt_decoder_, stt_vocab_)) {
            RCLCPP_ERROR(get_logger(), "Erreur chargement engines STT !");
            Pa_StopStream(mic); Pa_CloseStream(mic);
            return;
        }
        RCLCPP_INFO(get_logger(), "✅ STT prêt (Parakeet TDT, TRT)");

        // ── Warmup ──────────────────────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "🔥 Warmup TRT...");
        {
            std::vector<float> warmup(kSTTSampleRate, 0.0f);  // 1s silence
            asr.transcribe(warmup.data(), warmup.size());
        }
        RCLCPP_INFO(get_logger(), "✅ Warmup terminé — en écoute");

        // ── RNNoise + VAD ───────────────────────────────────────────────────
        RNNoise rnn;

        // vince : seuils RMS hysteresis identiques au Python
        constexpr float RMS_ON  = 0.02f;   // silence → parole
        constexpr float RMS_OFF = 0.015f;  // parole → silence

        constexpr float chunk_dur = static_cast<float>(kChunkSize) / kSTTSampleRate;
        const int kSilenceNeeded  = static_cast<int>(0.6f / chunk_dur);   // 600ms
        const int kMinSpeech      = static_cast<int>(0.2f / chunk_dur);   // 200ms
        const int kMaxSpeech      = static_cast<int>(10.0f / chunk_dur);  // 10s
        const int kPreBufMax      = static_cast<int>(0.256f / chunk_dur); // 256ms

        bool in_speech     = false;
        int  silence_count = 0;
        int  speech_count  = 0;

        std::vector<std::vector<float>> speech_buffer;
        std::vector<std::vector<float>> pre_buffer;

        int16_t raw[kChunkSize];
        float   chunk[kChunkSize];

        // ─────────────────────────────────────────────────────────────────────
        while (!g_killed) {

            err = Pa_ReadStream(mic, raw, kChunkSize);
            if (err != paNoError) continue;

            // int16 → float32
            for (int i = 0; i < kChunkSize; ++i)
                chunk[i] = raw[i] / 32768.0f;

            // RNNoise
            rnn.process(chunk, kChunkSize);

            // RMS hysteresis
            float rms = ComputeRMS(chunk, kChunkSize);
            float thr = in_speech ? RMS_OFF : RMS_ON;
            bool has_energy = (rms >= thr);

            // Copie chunk courant
            std::vector<float> cur(chunk, chunk + kChunkSize);

            if (has_energy) {
                silence_count = 0;
                speech_count++;
                speech_buffer.push_back(cur);

                if (!in_speech && speech_count >= kMinSpeech) {
                    in_speech = true;
                    // Préfixer avec le pre-buffer (évite de couper le début)
                    speech_buffer.insert(speech_buffer.begin(),
                                         pre_buffer.begin(), pre_buffer.end());
                    pre_buffer.clear();
                }

                if (in_speech) {
                    // vince : interrompre TTS dès que parole détectée
                    g_interrupt = true;
                    g_text_queue.cv.notify_all();
                }

                // Sécurité durée max
                if (speech_count >= kMaxSpeech) {
                    flush(asr, speech_buffer);
                    speech_buffer.clear(); pre_buffer.clear();
                    speech_count = silence_count = 0;
                    in_speech = false;
                }

            } else {
                if (in_speech) {
                    silence_count++;
                    speech_buffer.push_back(cur);

                    if (silence_count >= kSilenceNeeded) {
                        flush(asr, speech_buffer);
                        speech_buffer.clear(); pre_buffer.clear();
                        speech_count = silence_count = 0;
                        in_speech = false;
                    }
                } else {
                    // Ring buffer pré-parole
                    pre_buffer.push_back(cur);
                    if (static_cast<int>(pre_buffer.size()) > kPreBufMax)
                        pre_buffer.erase(pre_buffer.begin());
                    speech_buffer.clear();
                    speech_count = 0;
                }
            }
        }

        Pa_StopStream(mic);
        Pa_CloseStream(mic);
        asr.destroy();
        RCLCPP_INFO(get_logger(), "STT thread terminé");
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Concatène le buffer parole, transcrit, filtre et publie
    // ─────────────────────────────────────────────────────────────────────────
    void flush(ParakeetASR& asr,
               const std::vector<std::vector<float>>& buffer) {
        if (buffer.size() < 2) return;

        // Concaténer
        size_t total = 0;
        for (auto& c : buffer) total += c.size();
        std::vector<float> wav(total);
        size_t pos = 0;
        for (auto& c : buffer) {
            std::memcpy(wav.data() + pos, c.data(), c.size() * sizeof(float));
            pos += c.size();
        }

        float dur = static_cast<float>(wav.size()) / kSTTSampleRate;

        std::string text = asr.transcribe(wav.data(), static_cast<int>(wav.size()));

        // ── Filtres anti-hallucination (identiques au Python) ───────────────
        // Trim points/espaces
        auto ns = [](char c) { return c != ' ' && c != '.'; };
        auto s = std::find_if(text.begin(), text.end(), ns);
        auto e = std::find_if(text.rbegin(), text.rend(), ns).base();
        if (s >= e) return;
        text = std::string(s, e);

        // Au moins une lettre ASCII
        bool ascii_ok = false;
        for (char c : text)
            if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))
                { ascii_ok = true; break; }
        if (!ascii_ok || dur < 0.3f) return;

        RCLCPP_INFO(get_logger(), "💬 [%.2fs] → %s", dur, text.c_str());
        auto msg = std_msgs::msg::String();
        msg.data = text;
        pub_->publish(msg);
    }
};

// ═══════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenAndTalkNode>());
    rclcpp::shutdown();
    return 0;
}