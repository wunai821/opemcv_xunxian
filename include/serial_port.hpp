#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace xunji {

std::array<std::uint8_t, 8> makeVelocityFrame(
    std::int16_t velocity_x_mm_s, std::int16_t omega_mrad_s);

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud);
    bool writeVelocity(std::int16_t velocity_x_mm_s,
                       std::int16_t omega_mrad_s);
    void close();
    bool isOpen() const { return fd_ >= 0 && !write_failed_.load(); }

private:
    bool enqueueControl(std::string message);
    void writerLoop();

    int fd_ = -1;
    std::thread writer_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_ready_;
    std::string latest_control_;
    bool control_pending_ = false;
    bool stop_requested_ = false;
    std::atomic_bool write_failed_{false};
};

}  // namespace xunji
