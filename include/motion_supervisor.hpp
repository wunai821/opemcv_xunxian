#pragma once

#include "controller.hpp"
#include "line_follower.hpp"

#include <string>

namespace xunji {

struct MotionConfig {
    double max_velocity_m_s = 1.0;
    double max_omega_rad_s = 2.5;
    double max_accel_m_s2 = 0.8;
    double max_decel_m_s2 = 1.5;
    double max_omega_accel_rad_s2 = 8.0;
    double control_timeout_seconds = 0.20;
    double turn_velocity_m_s = 0.08;
    double turn_omega_rad_s = 1.5;
    double turn_duration_seconds = 1.0;
    double turn_timeout_seconds = 2.0;
    double straight_velocity_m_s = 0.20;
    double straight_node_seconds = 0.4;
    double straight_timeout_seconds = 1.2;
    int maneuver_reacquire_frames = 3;
    double reacquire_min_confidence = 0.35;
    double reacquire_max_steering = 0.60;
};

enum class MotionMode {
    Tracking,
    Maneuver,
    FaultStop
};

struct MotionOutput {
    double velocity_m_s = 0.0;
    double omega_rad_s = 0.0;
    MotionMode mode = MotionMode::Tracking;
    bool maneuver_completed = false;
    bool fault_triggered = false;
    std::string action;
    std::string fault_reason;
};

// 将视觉控制量、路口动作和安全约束统一仲裁后，才允许生成底盘 v、w。
class MotionSupervisor {
public:
    explicit MotionSupervisor(const MotionConfig& config);

    // 返回 false 表示已有动作正在执行，或系统已进入故障锁停。
    bool startManeuver(const std::string& action);
    MotionOutput update(const ControlCommand& tracking, bool line_found,
                        double line_confidence, RoadFeature feature,
                        double dt_seconds);
    void reset();

private:
    void enterFault(const std::string& reason);
    double limitVelocity(double target, double dt_seconds) const;
    double limitOmega(double target, double dt_seconds) const;

    MotionConfig config_;
    MotionMode mode_ = MotionMode::Tracking;
    std::string action_;
    std::string fault_reason_;
    double maneuver_elapsed_ = 0.0;
    int reacquire_frames_ = 0;
    double previous_velocity_ = 0.0;
    double previous_omega_ = 0.0;
    bool fault_report_pending_ = false;
    bool update_initialized_ = false;
};

const char* motionModeName(MotionMode mode);

}  // namespace xunji
