#include "mjpeg_server.hpp"

#include <opencv2/imgcodecs.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <sstream>
#include <string>
#include <utility>

namespace xunji {
namespace {

bool sendAll(int fd, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    std::size_t sent = 0;
    while (sent < size) {
        const ssize_t count =
            ::send(fd, bytes + sent, size - sent, MSG_NOSIGNAL);
        if (count > 0) {
            sent += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

bool sendText(int fd, const std::string& text) {
    return sendAll(fd, text.data(), text.size());
}

}  // namespace

MjpegServer::~MjpegServer() { stop(); }

bool MjpegServer::start(int port) {
    if (running_ || port < 1 || port > 65535) {
        return false;
    }

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        return false;
    }
    int reuse = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse,
                 sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) != 0 ||
        ::listen(server_fd_, 8) != 0) {
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    port_ = port;
    running_ = true;
    try {
        accept_thread_ = std::thread(&MjpegServer::acceptLoop, this);
    } catch (...) {
        running_ = false;
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }
    return true;
}

void MjpegServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    frame_ready_.notify_all();
    if (server_fd_ >= 0) {
        ::shutdown(server_fd_, SHUT_RDWR);
        ::close(server_fd_);
        server_fd_ = -1;
    }
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    std::vector<std::thread> clients;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients.swap(client_threads_);
    }
    for (auto& client : clients) {
        if (client.joinable()) {
            client.join();
        }
    }
    port_ = 0;
}

void MjpegServer::publish(const cv::Mat& bgr_frame, int jpeg_quality) {
    if (!running_ || !hasViewers() || bgr_frame.empty()) {
        return;
    }
    jpeg_quality = std::clamp(jpeg_quality, 30, 95);
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".jpg", bgr_frame, encoded,
                      {cv::IMWRITE_JPEG_QUALITY, jpeg_quality})) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_jpeg_ = std::move(encoded);
        ++frame_sequence_;
    }
    frame_ready_.notify_all();
}

void MjpegServer::acceptLoop() {
    while (running_) {
        sockaddr_in client_address{};
        socklen_t length = sizeof(client_address);
        const int client_fd = ::accept(
            server_fd_, reinterpret_cast<sockaddr*>(&client_address), &length);
        if (client_fd < 0) {
            if (running_ && errno == EINTR) {
                continue;
            }
            break;
        }

        timeval timeout{};
        timeout.tv_sec = 2;
        ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                     sizeof(timeout));
        ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                     sizeof(timeout));
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_threads_.emplace_back(&MjpegServer::handleClient, this,
                                     client_fd);
    }
}

void MjpegServer::handleClient(int client_fd) {
    std::array<char, 2048> request_buffer{};
    const ssize_t received =
        ::recv(client_fd, request_buffer.data(), request_buffer.size() - 1, 0);
    if (received <= 0) {
        ::close(client_fd);
        return;
    }
    const std::string request(request_buffer.data(),
                              static_cast<std::size_t>(received));

    if (request.rfind("GET /stream.mjpg ", 0) != 0) {
        static const std::string page =
            "<!doctype html><html><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
            "<title>OpenCV Xunji</title><style>body{margin:0;background:#111;"
            "color:#eee;font-family:sans-serif;text-align:center}h2{margin:12px}"
            "img{max-width:100vw;height:auto}</style></head><body>"
            "<h2>OpenCV Xunji Remote Preview</h2>"
            "<img src=\"/stream.mjpg\" alt=\"camera stream\"></body></html>";
        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                 << "Cache-Control: no-store\r\nContent-Length: "
                 << page.size() << "\r\nConnection: close\r\n\r\n" << page;
        sendText(client_fd, response.str());
        ::close(client_fd);
        return;
    }

    static const std::string stream_header =
        "HTTP/1.1 200 OK\r\n"
        "Cache-Control: no-store, no-cache, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    if (!sendText(client_fd, stream_header)) {
        ::close(client_fd);
        return;
    }

    ++stream_clients_;
    std::uint64_t seen_sequence = 0;
    while (running_) {
        std::vector<unsigned char> jpeg;
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_ready_.wait_for(lock, std::chrono::seconds(2), [&] {
                return !running_ || frame_sequence_ != seen_sequence;
            });
            if (!running_) {
                break;
            }
            if (frame_sequence_ == seen_sequence || latest_jpeg_.empty()) {
                continue;
            }
            jpeg = latest_jpeg_;
            seen_sequence = frame_sequence_;
        }

        std::ostringstream part_header;
        part_header << "--frame\r\nContent-Type: image/jpeg\r\n"
                    << "Content-Length: " << jpeg.size() << "\r\n\r\n";
        if (!sendText(client_fd, part_header.str()) ||
            !sendAll(client_fd, jpeg.data(), jpeg.size()) ||
            !sendText(client_fd, "\r\n")) {
            break;
        }
    }
    --stream_clients_;
    ::shutdown(client_fd, SHUT_RDWR);
    ::close(client_fd);
}

}  // namespace xunji
