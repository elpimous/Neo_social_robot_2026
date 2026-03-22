#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "qbo_arduqbo/cereal_port/CerealPort.h"

#define CEREAL_EXCEPT(except, msg, ...) \
  { char buf[1000]; \
    snprintf(buf, 1000, msg " (in cereal::CerealPort::%s)", ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); }

cereal::CerealPort::CerealPort() : fd_(-1), stream_paused_(false), stream_stopped_(true), stream_thread_running_(false) {}

cereal::CerealPort::~CerealPort() {
  if (portOpen()) close();
  stopStream();
}

// vince : conversion int → speed_t pour cfsetspeed()
// cfsetspeed() n'accepte pas un entier brut (ex: 115200) mais une constante speed_t
// encodée (ex: B115200 = 4098 en octal). Passer un int brut → baud rate incorrect.
// Le chip FTDI FT232 (vendor 0403) supporte nativement jusqu'à 921600 baud.
// ⚠️ B500000 et B921600 sont définis dans <termios.h> sur Linux mais pas sur tous les OS.
static speed_t cereal_to_speed_t(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 500000: return B500000;  // FTDI FT232 : 48MHz/96 = 500000, erreur 0%
        case 921600: return B921600;
        default:
            CEREAL_EXCEPT(cereal::Exception,
                "Baud rate %d non supporté — valeurs valides : 9600, 19200, 38400, "
                "57600, 115200, 230400, 500000, 921600", baud);
    }
}

void cereal::CerealPort::open(const char *port_name, int baud_rate) {
  if (portOpen()) close();

  fd_ = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (fd_ == -1) {
    const char *extra_msg = "";
    switch (errno) {
      case EACCES: extra_msg = "You probably don't have permission to open the port."; break;
      case ENOENT: extra_msg = "Port not found. Check the device."; break;
    }
    CEREAL_EXCEPT(cereal::Exception, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno), errno, extra_msg);
  }

  try {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();
    if (fcntl(fd_, F_SETLK, &fl) != 0)
      CEREAL_EXCEPT(cereal::Exception, "Device %s is already locked.", port_name);

    int status;
    ioctl(fd_, TIOCMGET, &status);
    status &= ~TIOCM_DTR;
    ioctl(fd_, TIOCMSET, &status);
    usleep(22);
    status |= TIOCM_DTR;
    ioctl(fd_, TIOCMSET, &status);

    struct termios newtio;
    tcgetattr(fd_, &newtio);
    memset(&newtio.c_cc, 0, sizeof(newtio.c_cc));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    // vince : utilisation de cereal_to_speed_t() au lieu de passer baud_rate brut
    // Ancienne ligne : cfsetspeed(&newtio, baud_rate) → comportement indéfini
    // Nouvelle ligne  : conversion explicite int → speed_t via switch/case
    cfsetspeed(&newtio, cereal_to_speed_t(baud_rate));
    baud_ = baud_rate;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) < 0)
      CEREAL_EXCEPT(cereal::Exception, "Unable to set port attributes: %s", port_name);
    usleep(200000);
  } catch (cereal::Exception &e) {
    if (fd_ != -1) ::close(fd_);
    fd_ = -1;
    throw e;
  }
}

void cereal::CerealPort::close() {
  if (fd_ != -1) {
    if (::close(fd_) != 0)
      CEREAL_EXCEPT(cereal::Exception, "Failed to close port properly: %s", strerror(errno));
    fd_ = -1;
  }
}

int cereal::CerealPort::write(const char *data, int length) {
  int len = length == -1 ? strlen(data) : length;
  int origflags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK);
  int retval = ::write(fd_, data, len);
  fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);
  if (retval == len) return retval;
  CEREAL_EXCEPT(cereal::Exception, "write failed");
}

int cereal::CerealPort::read(char *buffer, int max_length, int timeout) {
  struct pollfd ufd = { fd_, POLLIN, 0 };
  int retval = poll(&ufd, 1, timeout == 0 ? -1 : timeout);
  if (retval < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed: %s", strerror(errno));
  if (retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");
  if (ufd.revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "poll error");
  int ret = ::read(fd_, buffer, max_length);
  if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    CEREAL_EXCEPT(cereal::Exception, "read failed");
  return ret;
}

int cereal::CerealPort::readBytes(char *buffer, int length, int timeout) {
  int current = 0;
  while (current < length) {
    int r = read(&buffer[current], length - current, timeout);
    current += r;
  }
  return current;
}

int cereal::CerealPort::readLine(char *buffer, int length, int timeout) {
  int current = 0;
  while (current < length - 1) {
    if (current > 0 && buffer[current - 1] == '\n') return current;
    int r = read(&buffer[current], 1, timeout);
    current += r;
  }
  CEREAL_EXCEPT(cereal::Exception, "buffer full without newline");
}

bool cereal::CerealPort::readLine(std::string *buffer, int timeout) {
  buffer->clear();
  char c;
  while (buffer->size() < buffer->max_size() / 2) {
    if (read(&c, 1, timeout) > 0)
    buffer->push_back(c);
    if (c == '\n') return true;
  }
  CEREAL_EXCEPT(cereal::Exception, "line too long");
}

bool cereal::CerealPort::readBetween(std::string *buffer, char start, char end, int timeout) {
  buffer->clear();
  char c;
  bool started = false;
  while (buffer->size() < buffer->max_size() / 2) {
    if (read(&c, 1, timeout) <= 0) continue;
    if (!started) {
      if (c == start) {
        started = true;
        buffer->push_back(c);
      }
    } else {
      buffer->push_back(c);
      if (c == end) return true;
    }
  }
  CEREAL_EXCEPT(cereal::Exception, "did not reach end delimiter");
}

int cereal::CerealPort::flush() {
  int r = tcflush(fd_, TCIOFLUSH);
  if (r != 0) CEREAL_EXCEPT(cereal::Exception, "tcflush failed");
  return r;
}

bool cereal::CerealPort::startReadStream(std::function<void(char *, int)> f) {
  if (stream_thread_running_) return false;
  stream_stopped_ = false;
  stream_paused_ = false;
  readCallback = f;
  stream_thread_running_ = true;
  stream_thread_ = std::thread([this]() { this->readThread(); });
  return true;
}

bool cereal::CerealPort::startReadLineStream(std::function<void(std::string *)> f) {
  if (stream_thread_running_) return false;
  stream_stopped_ = false;
  stream_paused_ = false;
  readLineCallback = f;
  stream_thread_running_ = true;
  stream_thread_ = std::thread([this]() { this->readLineThread(); });
  return true;
}

bool cereal::CerealPort::startReadBetweenStream(std::function<void(std::string *)> f, char start, char end) {
  if (stream_thread_running_) return false;
  stream_stopped_ = false;
  stream_paused_ = false;
  readBetweenCallback = f;
  stream_thread_running_ = true;
  stream_thread_ = std::thread([this, start, end]() { this->readBetweenThread(start, end); });
  return true;
}

void cereal::CerealPort::readThread() {
  char data[MAX_LENGTH];
  while (!stream_stopped_) {
    if (!stream_paused_) {
      struct pollfd ufd = { fd_, POLLIN, 0 };
      if (poll(&ufd, 1, 10) > 0 && !(ufd.revents & POLLERR)) {
        int ret = ::read(fd_, data, MAX_LENGTH);
        if (ret > 0) readCallback(data, ret);
      }
    }
  }
  stream_thread_running_ = false;
}

void cereal::CerealPort::readLineThread() {
  std::string data;
  while (!stream_stopped_) {
    if (!stream_paused_) {
      try {
        if (readLine(&data, 100)) readLineCallback(&data);
      } catch (...) {}
    }
  }
  stream_thread_running_ = false;
}

void cereal::CerealPort::readBetweenThread(char start, char end) {
  std::string data;
  while (!stream_stopped_) {
    if (!stream_paused_) {
      try {
        if (readBetween(&data, start, end, 100)) readBetweenCallback(&data);
      } catch (...) {}
    }
  }
  stream_thread_running_ = false;
}

void cereal::CerealPort::stopStream() {
  stream_stopped_ = true;
  if (stream_thread_.joinable()) stream_thread_.join();
  stream_thread_running_ = false;
}

void cereal::CerealPort::pauseStream() { stream_paused_ = true; }
void cereal::CerealPort::resumeStream() { stream_paused_ = false; }

// EOF