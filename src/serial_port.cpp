#include "serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <utility>

namespace xunji {
namespace {

speed_t baudFlag(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B1000000
        case 1000000: return B1000000;
#endif
        default: return 0;
    }
}

std::uint16_t crc16Modbus(const std::uint8_t* data, std::size_t length) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U
                      ? static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U)
                      : static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

bool writeAll(int fd, const std::string& message) {
    std::size_t written = 0;
    while (written < message.size()) {
        const ssize_t result =
            ::write(fd, message.data() + written, message.size() - written);
        if (result > 0) {
            written += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

}  // namespace

std::array<std::uint8_t, 8> makeVelocityFrame(
    std::int16_t velocity_x_mm_s, std::int16_t omega_mrad_s) {
    const auto velocity = static_cast<std::uint16_t>(velocity_x_mm_s);
    const auto omega = static_cast<std::uint16_t>(omega_mrad_s);
    std::array<std::uint8_t, 8> frame{
        0xA5, 0x5A,
        static_cast<std::uint8_t>(velocity & 0xFFU),
        static_cast<std::uint8_t>((velocity >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(omega & 0xFFU),
        static_cast<std::uint8_t>((omega >> 8U) & 0xFFU),
        0, 0};
    const std::uint16_t crc = crc16Modbus(frame.data(), 6);
    frame[6] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame[7] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
    return frame;
}

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
    write_failed_ = false;
    const auto stop = makeVelocityFrame(0, 0);
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        latest_control_.assign(
            reinterpret_cast<const char*>(stop.data()), stop.size());
        control_pending_ = true;
        stop_requested_ = false;
    }
    try {
        writer_thread_ = std::thread(&SerialPort::writerLoop, this);
    } catch (...) {
        ::close(fd_);
        fd_ = -1;
        std::lock_guard<std::mutex> lock(queue_mutex_);
        latest_control_.clear();
        control_pending_ = false;
        return false;
    }
    return true;
}

bool SerialPort::writeVelocity(std::int16_t velocity_x_mm_s,
                               std::int16_t omega_mrad_s) {
    if (fd_ < 0) {
        return false;
    }
    const auto frame = makeVelocityFrame(velocity_x_mm_s, omega_mrad_s);
    return enqueueControl(std::string(
        reinterpret_cast<const char*>(frame.data()), frame.size()));
}

bool SerialPort::enqueueControl(std::string message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (fd_ < 0 || stop_requested_ || write_failed_) {
        return false;
    }
    // 控制量只保留最新一帧，串口短暂变慢时不积压过期转向指令。
    latest_control_ = std::move(message);
    control_pending_ = true;
    queue_ready_.notify_one();
    return true;
}

void SerialPort::writerLoop() {
    for (;;) {
        std::string message;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_ready_.wait(lock, [this] {
                return stop_requested_ || control_pending_;
            });
            if (control_pending_) {
                message = std::move(latest_control_);
                control_pending_ = false;
            } else if (stop_requested_) {
                break;
            }
        }
        if (!writeAll(fd_, message)) {
            write_failed_ = true;
            break;
        }
    }
}

void SerialPort::close() {
    if (writer_thread_.joinable()) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            // close 可能来自正常退出，也可能来自异常栈展开。只要物理写入
            // 尚未失败，就用停车帧覆盖所有待发控制量，再结束线程。
            if (!write_failed_) {
                const auto stop = makeVelocityFrame(0, 0);
                latest_control_.assign(
                    reinterpret_cast<const char*>(stop.data()), stop.size());
                control_pending_ = true;
            }
            stop_requested_ = true;
        }
        queue_ready_.notify_one();
        writer_thread_.join();
    }
    if (fd_ >= 0) {
        // writerLoop 返回只代表数据已交给内核；关闭前等待最后一帧真正发出，
        // 避免退出停车帧仍在驱动缓冲区时被 close 丢弃。
        while (::tcdrain(fd_) != 0 && errno == EINTR) {
        }
        ::close(fd_);
        fd_ = -1;
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    latest_control_.clear();
    control_pending_ = false;
    stop_requested_ = false;
}

}  // namespace xunji
