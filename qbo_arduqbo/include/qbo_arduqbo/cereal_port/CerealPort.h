#ifndef CEREAL_PORT_H
#define CEREAL_PORT_H

#include <stdexcept>
#include <termios.h>
#include <sys/ioctl.h>
#include <string>
#include <vector>
#include <thread>
#include <functional>
#include <atomic>

#define MAX_LENGTH 128

namespace cereal {

#define DEF_EXCEPTION(name, parent) \
  class name : public parent { \
  public: name(const char *msg) : parent(msg) {} }

  DEF_EXCEPTION(Exception, std::runtime_error);
  DEF_EXCEPTION(TimeoutException, Exception);
#undef DEF_EXCEPTION

  class CerealPort {
  public:
    CerealPort();
    ~CerealPort();

    void open(const char *port_name, int baud_rate = 115200);
    void close();
    bool portOpen() { return fd_ != -1; }
    int baudRate() { return baud_; }

    int write(const char *data, int length = -1);
    int read(char *data, int max_length, int timeout = -1);
    int readBytes(char *data, int length, int timeout = -1);
    int readLine(char *data, int length, int timeout = -1);
    bool readLine(std::string *data, int timeout = -1);
    bool readBetween(std::string *data, char start, char end, int timeout = -1);

    int flush();

    bool startReadStream(std::function<void(char *, int)> f);
    bool startReadLineStream(std::function<void(std::string *)> f);
    bool startReadBetweenStream(std::function<void(std::string *)> f, char start, char end);

    void stopStream();
    void pauseStream();
    void resumeStream();

  private:
    int fd_;
    int baud_;

    void readThread();
    void readLineThread();
    void readBetweenThread(char start, char end);

    std::thread stream_thread_;
    bool stream_paused_;
    bool stream_stopped_;
    std::atomic<bool> stream_thread_running_;
    std::function<void(char *, int)> readCallback;
    std::function<void(std::string *)> readLineCallback;
    std::function<void(std::string *)> readBetweenCallback;
  };

}

#endif
