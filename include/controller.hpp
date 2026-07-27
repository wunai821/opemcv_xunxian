#pragma once

namespace xunji {

struct ControllerConfig {
    double kp = 0.85;
    double ki = 0.0;
    double kd = 0.12;
    double max_steer = 1.0;
    double base_speed = 0.35;  // m/s
    double min_speed = 0.16;   // m/s
    double lost_stop_seconds = 0.20;
};

struct ControlCommand {
    double steering = 0.0;  // -1 左，+1 右
    double speed = 0.0;     // 前进速度，m/s
    bool line_lost = true;
};

class Controller {
public:
    explicit Controller(const ControllerConfig& config);
    ControlCommand update(double error, double confidence, double curvature,
                          double dt_seconds);
    void reset();

private:
    ControllerConfig config_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    double lost_time_ = 0.0;
    bool initialized_ = false;
};

}  // namespace xunji
