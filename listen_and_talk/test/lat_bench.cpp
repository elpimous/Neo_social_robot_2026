/**
 * @file    lat_bench.cpp
 * @brief   Banc de test hors ROS des moteurs listen / talk.
 *
 *  lat_bench tts      <dossier_axel> "texte" sortie.wav
 *  lat_bench asr      <dossier_parakeet> <vad.onnx> fichier.wav     (16 kHz mono)
 *  lat_bench scenario <dossier_axel> <dossier_parakeet> <vad.onnx> [crête_écho]
 *
 *  "scenario" simule : question utilisateur, phrases du robot captées en
 *  écho (doivent être ignorées), barge-in de l'utilisateur pendant une
 *  phrase du robot, réponse rapide juste après la fin d'une phrase.
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>

#include "listen_and_talk/listen_engine.hpp"
#include "listen_and_talk/talk_engine.hpp"

using namespace lat;

static LogFn kLog = [](int l, const std::string& s) {
  std::printf("  [%s] %s\n", l == 0 ? "info" : (l == 1 ? "warn" : "ERR "), s.c_str());
};

static void write_wav(const std::string& p, const std::vector<float>& x, int sr) {
  std::ofstream f(p, std::ios::binary);
  const int32_t n = static_cast<int32_t>(x.size()), br = sr * 2, ds = n * 2, rs = 36 + ds, fs = 16;
  const int16_t fmt = 1, ch = 1, ba = 2, bps = 16;
  f.write("RIFF", 4); f.write((const char*)&rs, 4); f.write("WAVEfmt ", 8);
  f.write((const char*)&fs, 4); f.write((const char*)&fmt, 2); f.write((const char*)&ch, 2);
  f.write((const char*)&sr, 4); f.write((const char*)&br, 4); f.write((const char*)&ba, 2);
  f.write((const char*)&bps, 2); f.write("data", 4); f.write((const char*)&ds, 4);
  for (float v : x) {
    const int16_t s = static_cast<int16_t>(std::lrint(std::clamp(v, -1.f, 1.f) * 32767));
    f.write((const char*)&s, 2);
  }
}

// Lecteur WAV PCM16 minimal (mono ou multi-canal → canal 0).
static bool read_wav(const std::string& p, std::vector<float>& x, int& sr) {
  std::ifstream f(p, std::ios::binary);
  if (!f) return false;
  char id[4];
  int32_t sz;
  f.read(id, 4); f.read((char*)&sz, 4); f.read(id, 4);
  int16_t ch = 1, bps = 16;
  while (f.read(id, 4) && f.read((char*)&sz, 4)) {
    if (!std::strncmp(id, "fmt ", 4)) {
      int16_t fmt; int32_t br; int16_t ba;
      f.read((char*)&fmt, 2); f.read((char*)&ch, 2); f.read((char*)&sr, 4);
      f.read((char*)&br, 4); f.read((char*)&ba, 2); f.read((char*)&bps, 2);
      f.seekg(sz - 16, std::ios::cur);
    } else if (!std::strncmp(id, "data", 4)) {
      if (bps != 16) return false;
      std::vector<int16_t> raw(sz / 2);
      f.read((char*)raw.data(), sz);
      for (size_t i = 0; i < raw.size(); i += ch) x.push_back(raw[i] / 32768.0f);
      return true;
    } else {
      f.seekg(sz, std::ios::cur);
    }
  }
  return false;
}

static TalkEngine::Config tts_cfg(const std::string& d) {
  TalkEngine::Config c;
  c.model = d + "/axel.onnx";
  c.tokens = d + "/tokens.txt";
  c.data_dir = d + "/espeak-ng-data";
  c.provider = std::getenv("LAT_PROVIDER") ? std::getenv("LAT_PROVIDER") : "cpu";
  c.num_threads = 4;
  return c;
}

static AsrEngine::Config asr_cfg(const std::string& d) {
  AsrEngine::Config c;
  c.encoder = d + "/encoder.int8.onnx";
  c.decoder = d + "/decoder.int8.onnx";
  c.joiner = d + "/joiner.int8.onnx";
  c.tokens = d + "/tokens.txt";
  c.provider = std::getenv("LAT_PROVIDER") ? std::getenv("LAT_PROVIDER") : "cpu";
  c.num_threads = 4;
  return c;
}

static std::vector<float> say16(TalkEngine& e, const std::string& t, float pk) {
  std::vector<float> a, o;
  e.synthesize_to_buffer(t, a);
  PolyphaseResampler r(e.model_sample_rate(), 16000);
  r.process(a.data(), a.size(), o);
  r.flush(o);
  const float p = peak(o.data(), o.size());
  for (auto& v : o) v *= pk / p;
  return o;
}

static void print_out(const ListenEngine::Output& o) {
  std::printf(">>> %s   [conf %.2f | audio %.2f s | calcul %.0f ms | RTF %.3f%s]\n", o.text.c_str(),
              o.confidence, o.duration_s, o.compute_ms, o.compute_ms / 1000.0 / o.duration_s,
              o.after_barge_in ? " | barge-in" : "");
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "voir l'en-tête de lat_bench.cpp pour l'usage\n");
    return 1;
  }
  const std::string mode = argv[1];

  if (mode == "tts" && argc >= 5) {
    TalkEngine e(tts_cfg(argv[2]), nullptr, kLog);
    if (!e.init()) return 1;
    std::vector<float> out;
    e.set_test_sink([&](const float* p, size_t n) { out.insert(out.end(), p, p + n); }, 44100);
    e.start();
    e.say(argv[3]);
    while (e.busy()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    e.stop();
    write_wav(argv[4], out, 44100);
    const auto s = e.last_stats();
    std::printf("1er son %.0f ms | synthèse %.0f ms | audio %.0f ms | RTF %.3f\n", s.first_audio_ms,
                s.synth_ms, s.audio_ms, s.synth_ms / s.audio_ms);
    return 0;
  }

  if (mode == "asr" && argc >= 5) {
    std::vector<float> x;
    int sr = 0;
    if (!read_wav(argv[4], x, sr)) return 1;
    if (sr != 16000) {
      std::vector<float> y;
      PolyphaseResampler r(sr, 16000);
      r.process(x.data(), x.size(), y);
      r.flush(y);
      x.swap(y);
    }
    ListenPipeline::Config pc;
    pc.vad_model = argv[3];
    ListenEngine le(nullptr, pc, asr_cfg(argv[2]), TextFilter{}, kLog, print_out, nullptr);
    if (!le.init()) return 1;
    le.start();
    x.insert(x.end(), 16000, 0.0f);  // 1 s de silence final pour clore le dernier segment
    le.feed(x.data(), x.size());
    le.drain();
    le.stop();
    return 0;
  }

  if (mode == "scenario" && argc >= 5) {
    const float echo_pk = argc >= 6 ? std::atof(argv[5]) : 0.15f;
    TalkEngine tts(tts_cfg(argv[2]), nullptr, nullptr);
    if (!tts.init()) return 1;
    const std::vector<std::string> robot = {
        "Je vais très bien, merci. Il fait beau dans le salon.",
        "Hum. AH ! Attention, je vais te raconter une très longue histoire à propos des étoiles et des planètes !",
        "Bonjour Vincent ! Tu veux jouer au ballon rouge ? Je peux aussi tourner la tête et te suivre du regard.",
        "D'accord.",
        "Il est quinze heures trente. La batterie est chargée à quatre-vingt pour cent."};
    const auto UA = say16(tts, "Bonjour Néo, comment vas-tu aujourd'hui ?", 0.5f);
    const auto UB = say16(tts, "Stop, arrête de parler s'il te plaît.", 0.6f);
    const auto UC = say16(tts, "Oui, je veux bien jouer au ballon.", 0.5f);

    ListenPipeline::Config pc;
    pc.vad_model = argv[4];
    int barge = 0, texts = 0;
    bool fired = false;
    ListenEngine le(nullptr, pc, asr_cfg(argv[3]), TextFilter{}, kLog,
                    [&](const ListenEngine::Output& o) { ++texts; print_out(o); },
                    [&] { ++barge; fired = true; });
    if (!le.init()) return 1;
    le.start();

    std::mt19937 g(1);
    std::normal_distribution<float> noise(0.0f, 0.002f);
    std::vector<float> blk(512);
    // Simulation temps réel : le robot se tait 100 ms après un barge-in.
    auto run = [&](const std::vector<float>& echo, const std::vector<float>* user, size_t uoff,
                   double after_s) {
      fired = false;
      size_t stop_at = SIZE_MAX;
      const size_t total = std::max(echo.size(), user ? uoff + user->size() : 0) +
                           static_cast<size_t>(after_s * 16000);
      for (size_t i = 0; i < total; i += 512) {
        const bool spk = i < echo.size() && i < stop_at;
        for (int k = 0; k < 512; ++k) {
          const size_t j = i + k;
          float v = noise(g);
          if (j < echo.size() && j < stop_at) v += echo[j];
          if (user && j >= uoff && j - uoff < user->size()) v += (*user)[j - uoff];
          blk[k] = v;
        }
        le.pipeline().set_speaking(spk);
        le.feed(blk.data(), 512);
        if (fired && stop_at == SIZE_MAX) stop_at = i + 1600;
      }
    };
    std::printf("── Question utilisateur\n");
    run({}, &UA, 16000, 1.5);
    le.drain();
    for (size_t r = 0; r < robot.size(); ++r) {
      const auto E = say16(tts, robot[r], echo_pk);
      if (r == 1) {
        std::printf("── Robot #%zu + barge-in utilisateur\n", r);
        run(E, &UB, 24000, 1.5);
      } else if (r == 4) {
        std::printf("── Robot #%zu puis réponse rapide (200 ms après)\n", r);
        run(E, nullptr, 0, 0);
        run({}, &UC, 3200, 1.5);
      } else {
        std::printf("── Robot #%zu seul (écho)\n", r);
        run(E, nullptr, 0, 1.0);
      }
      le.drain();
    }
    le.stop();
    std::printf("\nRÉSULTAT : %d textes (attendu 3), %d barge-in (attendu 1)\n", texts, barge);
    return (texts == 3 && barge == 1) ? 0 : 2;
  }

  std::fprintf(stderr, "arguments invalides\n");
  return 1;
}
