#include "serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool checkFrame(std::int16_t velocity, std::int16_t omega,
                const std::array<std::uint8_t, 8>& expected) {
    if (xunji::makeVelocityFrame(velocity, omega) != expected) {
        std::cerr << "FAIL: protocol frame mismatch for v=" << velocity
                  << " w=" << omega << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= checkFrame(300, 0, {0xA5, 0x5A, 0x2C, 0x01,
                              0x00, 0x00, 0x19, 0xB3});
    ok &= checkFrame(0, 1500, {0xA5, 0x5A, 0x00, 0x00,
                               0xDC, 0x05, 0xD8, 0x20});
    ok &= checkFrame(0, -1500, {0xA5, 0x5A, 0x00, 0x00,
                                0x24, 0xFA, 0xDB, 0xA0});
    ok &= checkFrame(0, 0, {0xA5, 0x5A, 0x00, 0x00,
                            0x00, 0x00, 0x40, 0xE3});
    if (!ok) return 1;

    const int master_fd = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd < 0 || ::grantpt(master_fd) != 0 ||
        ::unlockpt(master_fd) != 0) {
        std::cerr << "FAIL: cannot create pseudo terminal\n";
        if (master_fd >= 0) ::close(master_fd);
        return 1;
    }
    const char* slave_name = ::ptsname(master_fd);
    if (slave_name == nullptr) {
        std::cerr << "FAIL: cannot resolve pseudo terminal slave\n";
        ::close(master_fd);
        return 1;
    }

    xunji::SerialPort serial;
    if (!serial.open(slave_name, 1000000)) {
        std::cerr << "FAIL: cannot open pseudo serial port\n";
        ::close(master_fd);
        return 1;
    }

    // 快速生产大量控制量，验证调用方不会等待物理串口逐条发送。
    for (int index = 0; index < 500; ++index) {
        if (!serial.writeVelocity(250, static_cast<std::int16_t>(index))) {
            std::cerr << "FAIL: cannot enqueue control command\n";
            ::close(master_fd);
            return 1;
        }
    }
    // 不显式发送停车命令：验证 close/异常栈展开自身会覆盖待发控制量，
    // 并把停车帧作为最后一帧发出。
    serial.close();

    std::string received;
    char buffer[4096];
    for (;;) {
        const ssize_t size = ::read(master_fd, buffer, sizeof(buffer));
        if (size > 0) {
            received.append(buffer, static_cast<std::size_t>(size));
            continue;
        }
        if (size < 0 && errno == EINTR) continue;
        if (size == 0 || (size < 0 && errno == EIO)) break;
        std::cerr << "FAIL: cannot read pseudo terminal\n";
        ::close(master_fd);
        return 1;
    }
    ::close(master_fd);

    const auto stop = xunji::makeVelocityFrame(0, 0);
    const bool valid_length = !received.empty() && received.size() % 8 == 0;
    const bool stop_received = received.size() >= stop.size() &&
        std::equal(stop.begin(), stop.end(), received.end() - stop.size(),
                   [](std::uint8_t expected, char actual) {
                       return expected == static_cast<std::uint8_t>(actual);
                   });
    if (!valid_length || !stop_received) {
        std::cerr << "FAIL: asynchronous serial queue lost final stop frame"
                  << " size=" << received.size() << " bytes=";
        for (const unsigned char byte : received) {
            std::cerr << std::hex << static_cast<int>(byte) << ' ';
        }
        std::cerr << '\n';
        return 1;
    }
    return 0;
}
