#include "serial_port.hpp"

#include "line_follower.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>

namespace xunji {
namespace {

speed_t baudFlag(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return 0;
    }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud) {
    close();
    const speed_t speed = baudFlag(baud);
    if (speed == 0) {
        return false;
    }
    fd_ = ::open(device.c_str(), O_WRONLY | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }
    return true;
}

bool SerialPort::writeCommand(double steering, double speed,
                              double confidence) {
    if (fd_ < 0) {
        return false;
    }
    char message[96];
    const int length = std::snprintf(message, sizeof(message),
                                     "$CTRL,%.3f,%.3f,%.3f\n",
                                     steering, speed, confidence);
    return length > 0 &&
           ::write(fd_, message, static_cast<std::size_t>(length)) == length;
}

bool SerialPort::writeNode(RoadFeature feature, const std::string& action,
                           int exits_mask, double confidence) {
    if (fd_ < 0) {
        return false;
    }
    char message[128];
    const int length = std::snprintf(
        message, sizeof(message), "$NODE,%s,%s,%d,%.3f\n",
        featureName(feature), action.c_str(), exits_mask, confidence);
    return length > 0 &&
           ::write(fd_, message, static_cast<std::size_t>(length)) == length;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

}  // namespace xunji
