#pragma once

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace xunji {

// 轻量级只读 HTTP/MJPEG 服务，用于无桌面环境远程查看调试画面。
class MjpegServer {
public:
    MjpegServer() = default;
    ~MjpegServer();
    MjpegServer(const MjpegServer&) = delete;
    MjpegServer& operator=(const MjpegServer&) = delete;

    bool start(int port);
    void stop();
    void publish(const cv::Mat& bgr_frame, int jpeg_quality = 75);

    bool isRunning() const { return running_.load(); }
    bool hasViewers() const { return stream_clients_.load() > 0; }
    int port() const { return port_; }

private:
    void acceptLoop();
    void handleClient(int client_fd);

    int server_fd_ = -1;
    int port_ = 0;
    std::atomic_bool running_{false};
    std::atomic_int stream_clients_{0};
    std::thread accept_thread_;

    std::mutex clients_mutex_;
    std::vector<std::thread> client_threads_;

    std::mutex frame_mutex_;
    std::condition_variable frame_ready_;
    std::vector<unsigned char> latest_jpeg_;
    std::uint64_t frame_sequence_ = 0;
};

}  // namespace xunji
