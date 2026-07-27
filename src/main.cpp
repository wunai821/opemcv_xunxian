#include "controller.hpp"
#include "line_follower.hpp"
#include "motion_supervisor.hpp"
#include "serial_port.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::atomic_bool running{true};
void stopHandler(int) { running = false; }

struct Options {
    std::string config = "config/default.yaml";
    std::string input = "0";
    std::string serial_device;
    bool headless = false;
    int max_frames = 0;
};

std::vector<std::string> parseRoute(const std::string& text) {
    std::vector<std::string> route;
    std::stringstream stream(text);
    std::string action;
    while (std::getline(stream, action, ',')) {
        const auto first = action.find_first_not_of(" \t");
        const auto last = action.find_last_not_of(" \t");
        if (first != std::string::npos) {
            action = action.substr(first, last - first + 1);
            if (action == "LEFT" || action == "RIGHT" ||
                action == "STRAIGHT") {
                route.push_back(action);
            }
        }
    }
    if (route.empty()) {
        route.push_back("STRAIGHT");
    }
    return route;
}

bool actionAvailable(const std::string& action, const xunji::PathExits& exits) {
    return (action == "LEFT" && exits.left) ||
           (action == "RIGHT" && exits.right) ||
           (action == "STRAIGHT" && exits.top);
}

std::string chooseAction(const xunji::LineResult& line,
                         const std::vector<std::string>& route,
                         std::size_t route_index) {
    if (line.feature == xunji::RoadFeature::CornerLeft) {
        return "LEFT";
    }
    if (line.feature == xunji::RoadFeature::CornerRight) {
        return "RIGHT";
    }

    const std::string planned = route[route_index % route.size()];
    if (actionAvailable(planned, line.exits)) {
        return planned;
    }
    // 规划方向在当前节点不存在时，优先直行，再选择左/右，避免发出无效动作。
    if (line.exits.top) return "STRAIGHT";
    if (line.exits.left) return "LEFT";
    if (line.exits.right) return "RIGHT";
    return "STOP";
}

void usage(const char* program) {
    std::cout
        << "用法: " << program << " [选项]\n"
        << "  --config PATH      YAML 配置文件\n"
        << "  --camera N         V4L2 摄像头序号（默认 0）\n"
        << "  --video PATH       使用视频文件\n"
        << "  --serial DEVICE    输出控制指令到串口，例如 /dev/ttyS3\n"
        << "  --headless         不创建显示窗口\n"
        << "  --max-frames N     处理 N 帧后退出（0 为持续运行）\n"
        << "  --help             显示帮助\n";
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) {
                throw std::runtime_error(arg + " 缺少参数");
            }
            return argv[i];
        };
        if (arg == "--config") options.config = value();
        else if (arg == "--camera") options.input = value();
        else if (arg == "--video") options.input = value();
        else if (arg == "--serial") options.serial_device = value();
        else if (arg == "--headless") options.headless = true;
        else if (arg == "--max-frames") options.max_frames = std::stoi(value());
        else if (arg == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("未知参数: " + arg);
        }
    }
    return options;
}

template <typename T>
void readValue(const cv::FileStorage& fs, const char* name, T& value) {
    if (!fs[name].empty()) {
        fs[name] >> value;
    }
}

void loadConfig(const std::string& path, xunji::VisionConfig& vision,
                xunji::ControllerConfig& control, int& camera_width,
                int& camera_height, int& camera_fps, int& serial_baud,
                std::string& route, xunji::MotionConfig& motion) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("无法打开配置文件: " + path);
    }
    readValue(fs, "camera_width", camera_width);
    readValue(fs, "camera_height", camera_height);
    readValue(fs, "camera_fps", camera_fps);
    readValue(fs, "process_width", vision.process_width);
    readValue(fs, "process_height", vision.process_height);
    readValue(fs, "roi_top_ratio", vision.roi_top_ratio);
    readValue(fs, "roi_bottom_ratio", vision.roi_bottom_ratio);
    int dark_line = vision.dark_line ? 1 : 0;
    readValue(fs, "dark_line", dark_line);
    vision.dark_line = dark_line != 0;
    readValue(fs, "blur_size", vision.blur_size);
    readValue(fs, "morph_size", vision.morph_size);
    readValue(fs, "min_track_width", vision.min_track_width);
    readValue(fs, "max_track_width", vision.max_track_width);
    readValue(fs, "search_radius", vision.search_radius);
    readValue(fs, "max_missing_rows", vision.max_missing_rows);
    readValue(fs, "center_smoothing", vision.center_smoothing);
    readValue(fs, "probe_inset_ratio", vision.probe_inset_ratio);
    readValue(fs, "probe_min_run", vision.probe_min_run);
    readValue(fs, "feature_confirm_frames", vision.feature_confirm_frames);
    readValue(fs, "feature_clear_frames", vision.feature_clear_frames);
    readValue(fs, "route", route);
    readValue(fs, "kp", control.kp);
    readValue(fs, "ki", control.ki);
    readValue(fs, "kd", control.kd);
    readValue(fs, "max_steer", control.max_steer);
    readValue(fs, "base_speed", control.base_speed);
    readValue(fs, "min_speed", control.min_speed);
    readValue(fs, "lost_stop_seconds", control.lost_stop_seconds);
    readValue(fs, "max_velocity_m_s", motion.max_velocity_m_s);
    readValue(fs, "max_omega_rad_s", motion.max_omega_rad_s);
    readValue(fs, "max_accel_m_s2", motion.max_accel_m_s2);
    readValue(fs, "max_decel_m_s2", motion.max_decel_m_s2);
    readValue(fs, "max_omega_accel_rad_s2",
              motion.max_omega_accel_rad_s2);
    readValue(fs, "control_timeout_seconds",
              motion.control_timeout_seconds);
    readValue(fs, "turn_velocity_m_s", motion.turn_velocity_m_s);
    readValue(fs, "turn_omega_rad_s", motion.turn_omega_rad_s);
    readValue(fs, "turn_duration_seconds", motion.turn_duration_seconds);
    readValue(fs, "turn_timeout_seconds", motion.turn_timeout_seconds);
    readValue(fs, "straight_velocity_m_s", motion.straight_velocity_m_s);
    readValue(fs, "straight_node_seconds", motion.straight_node_seconds);
    readValue(fs, "straight_timeout_seconds",
              motion.straight_timeout_seconds);
    readValue(fs, "maneuver_reacquire_frames",
              motion.maneuver_reacquire_frames);
    readValue(fs, "reacquire_min_confidence",
              motion.reacquire_min_confidence);
    readValue(fs, "reacquire_max_steering",
              motion.reacquire_max_steering);
    readValue(fs, "serial_baud", serial_baud);
}

std::int16_t clampInt16(double value, int minimum, int maximum) {
    return static_cast<std::int16_t>(
        std::clamp(static_cast<int>(std::lround(value)), minimum, maximum));
}

bool isCameraIndex(const std::string& input) {
    return !input.empty() &&
           input.find_first_not_of("0123456789") == std::string::npos;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        xunji::VisionConfig vision_config;
        xunji::ControllerConfig control_config;
        int camera_width = 640;
        int camera_height = 480;
        int camera_fps = 30;
        int serial_baud = 1000000;
        std::string route_text = "STRAIGHT";
        xunji::MotionConfig motion_config;
        loadConfig(options.config, vision_config, control_config, camera_width,
                   camera_height, camera_fps, serial_baud, route_text,
                   motion_config);
        const std::vector<std::string> route = parseRoute(route_text);

        cv::VideoCapture capture;
        if (isCameraIndex(options.input)) {
            capture.open(std::stoi(options.input), cv::CAP_V4L2);
            capture.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
            capture.set(cv::CAP_PROP_FPS, camera_fps);
            capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
        } else {
            capture.open(options.input);
        }
        if (!capture.isOpened()) {
            throw std::runtime_error("无法打开输入: " + options.input);
        }

        xunji::SerialPort serial;
        if (!options.serial_device.empty() &&
            !serial.open(options.serial_device, serial_baud)) {
            throw std::runtime_error("无法打开串口: " + options.serial_device);
        }

        xunji::LineFollower follower(vision_config);
        xunji::Controller controller(control_config);
        xunji::MotionSupervisor motion(motion_config);
        std::signal(SIGINT, stopHandler);
        std::signal(SIGTERM, stopHandler);
        auto previous_time = std::chrono::steady_clock::now();
        constexpr auto status_interval = std::chrono::milliseconds(100);
        auto previous_status_time = previous_time - status_interval;
        int frame_count = 0;
        std::size_t route_index = 0;

        while (running && (options.max_frames <= 0 ||
                           frame_count < options.max_frames)) {
            cv::Mat frame;
            if (!capture.read(frame)) {
                break;
            }
            const auto now = std::chrono::steady_clock::now();
            const double dt =
                std::chrono::duration<double>(now - previous_time).count();
            previous_time = now;

            const xunji::LineResult line = follower.process(frame);
            const xunji::ControlCommand command =
                controller.update(line.error, line.found ? line.confidence : 0.0,
                                  line.curvature, dt);
            if (line.feature_confirmed) {
                const std::string action =
                    chooseAction(line, route, route_index);
                std::cout << "\n节点确认: " << xunji::featureName(line.feature)
                          << " action=" << action
                          << " exits=" << line.exits.mask() << '\n';
                if (!motion.startManeuver(action)) {
                    std::cerr << "节点动作未启动，当前运动状态不允许新动作\n";
                } else if (line.feature != xunji::RoadFeature::CornerLeft &&
                           line.feature != xunji::RoadFeature::CornerRight) {
                    ++route_index;
                }
            }

            const xunji::MotionOutput output = motion.update(
                command, line.found, line.confidence, line.feature, dt);
            if (output.maneuver_completed) {
                controller.reset();
                std::cout << "\n节点动作完成: " << output.action
                          << "，已重新找到赛道\n";
            }
            if (output.fault_triggered) {
                std::cerr << "\n运动控制锁停: " << output.fault_reason
                          << "；需排除问题并重启程序\n";
            }

            const std::int16_t velocity_x_mm_s =
                clampInt16(output.velocity_m_s * 1000.0, -1000, 1000);
            const std::int16_t omega_mrad_s =
                clampInt16(output.omega_rad_s * 1000.0, -6000, 6000);
            if (!options.serial_device.empty() &&
                !serial.writeVelocity(velocity_x_mm_s, omega_mrad_s)) {
                throw std::runtime_error(
                    "串口发送失败，底盘将由 200 ms 通信看门狗停车");
            }

            // 终端刷新限制到 10 Hz，避免 SSH/串口终端输出拖慢视觉主循环。
            if (now - previous_status_time >= status_interval) {
                previous_status_time = now;
                std::cout << std::fixed << std::setprecision(3)
                          << "\rerror=" << line.error
                          << " confidence=" << line.confidence
                          << " road=" << xunji::featureName(line.feature)
                          << " steer=" << command.steering
                          << " v=" << velocity_x_mm_s << "mm/s"
                          << " w=" << omega_mrad_s << "mrad/s"
                          << " mode=" << xunji::motionModeName(output.mode)
                          << (output.action.empty()
                                  ? ""
                                  : " action=" + output.action)
                          << "   "
                          << std::flush;
            }

            if (!options.headless) {
                cv::Mat debug = follower.drawDebug(frame, line);
                cv::imshow("opencv_xunji", debug);
                cv::imshow("binary", line.binary);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q') {
                    break;
                }
            }
            ++frame_count;
        }

        if (serial.isOpen()) {
            serial.writeVelocity(0, 0);
        }
        std::cout << "\n已停止，共处理 " << frame_count << " 帧\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 1;
    }
}
