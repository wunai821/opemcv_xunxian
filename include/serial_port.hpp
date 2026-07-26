#pragma once

#include <string>

namespace xunji {

enum class RoadFeature;

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud);
    bool writeCommand(double steering, double speed, double confidence);
    bool writeNode(RoadFeature feature, const std::string& action,
                   int exits_mask, double confidence);
    void close();
    bool isOpen() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

}  // namespace xunji
