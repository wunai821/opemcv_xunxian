#include "motion_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xunji {
namespace {

double approach(double current, double target, double maximum_change) {
    return current + std::clamp(target - current, -maximum_change,
                                maximum_change);
}

}  // namespace

const char* motionModeName(MotionMode mode) {
    switch (mode) {
        case MotionMode::Tracking: return "TRACKING";
        case MotionMode::Maneuver: return "MANEUVER";
        case MotionMode::FaultStop: return "FAULT_STOP";
    }
    return "UNKNOWN";
}

MotionSupervisor::MotionSupervisor(const MotionConfig& config)
    : config_(config) {
    if (config_.max_velocity_m_s <= 0.0 ||
        config_.max_omega_rad_s <= 0.0 ||
        config_.max_accel_m_s2 <= 0.0 || config_.max_decel_m_s2 <= 0.0 ||
        config_.max_omega_accel_rad_s2 <= 0.0 ||
        config_.control_timeout_seconds <= 0.0 ||
        config_.turn_velocity_m_s < 0.0 ||
        config_.turn_omega_rad_s <= 0.0 ||
        config_.turn_duration_seconds < 0.0 ||
        config_.turn_timeout_seconds <= config_.turn_duration_seconds ||
        config_.straight_velocity_m_s < 0.0 ||
        config_.straight_node_seconds < 0.0 ||
        config_.straight_timeout_seconds <= config_.straight_node_seconds ||
        config_.maneuver_reacquire_frames <= 0 ||
        config_.reacquire_min_confidence < 0.0 ||
        config_.reacquire_min_confidence > 1.0 ||
        config_.reacquire_max_steering < 0.0 ||
        config_.reacquire_max_steering > 1.0 ||
        config_.max_velocity_m_s > 1.0 ||
        config_.max_omega_rad_s > 6.0 ||
        config_.turn_velocity_m_s > config_.max_velocity_m_s ||
        config_.straight_velocity_m_s > config_.max_velocity_m_s ||
        config_.turn_omega_rad_s > config_.max_omega_rad_s) {
        throw std::invalid_argument("运动控制参数非法");
    }
}

bool MotionSupervisor::startManeuver(const std::string& action) {
    if (mode_ == MotionMode::FaultStop) {
        return false;
    }
    if (mode_ == MotionMode::Maneuver) {
        enterFault("上一个节点动作未结束，又检测到新节点");
        return false;
    }
    if (action != "LEFT" && action != "RIGHT" && action != "STRAIGHT") {
        enterFault("节点没有可执行方向");
        return false;
    }
    mode_ = MotionMode::Maneuver;
    action_ = action;
    maneuver_elapsed_ = 0.0;
    reacquire_frames_ = 0;
    return true;
}

MotionOutput MotionSupervisor::update(const ControlCommand& tracking,
                                      bool line_found, double line_confidence,
                                      RoadFeature feature,
                                      double dt_seconds) {
    if (mode_ != MotionMode::FaultStop && update_initialized_ &&
        dt_seconds > config_.control_timeout_seconds) {
        enterFault("视觉控制周期超时");
    }
    update_initialized_ = true;
    dt_seconds = std::clamp(dt_seconds, 0.001, 0.2);
    MotionOutput output;

    const double tracking_velocity =
        std::clamp(tracking.speed, 0.0, config_.max_velocity_m_s);
    const double tracking_omega = std::clamp(
        -tracking.steering * config_.max_omega_rad_s,
        -config_.max_omega_rad_s, config_.max_omega_rad_s);
    double target_velocity = 0.0;
    double target_omega = 0.0;
    if (mode_ == MotionMode::Tracking) {
        target_velocity = tracking_velocity;
        target_omega = tracking_omega;
    } else if (mode_ == MotionMode::Maneuver) {
        maneuver_elapsed_ += dt_seconds;
        const bool turning = action_ == "LEFT" || action_ == "RIGHT";
        const double minimum_duration =
            turning ? config_.turn_duration_seconds
                    : config_.straight_node_seconds;
        const double timeout = turning ? config_.turn_timeout_seconds
                                       : config_.straight_timeout_seconds;

        target_velocity = turning ? config_.turn_velocity_m_s
                                  : config_.straight_velocity_m_s;
        target_omega = turning
                           ? (action_ == "LEFT" ? config_.turn_omega_rad_s
                                                : -config_.turn_omega_rad_s)
                           : 0.0;

        // 固定时长只作为最短动作时间；随后必须连续看到普通赛道，
        // 才认为真正驶离路口并恢复视觉闭环。
        constexpr double time_epsilon = 1e-9;
        const bool reacquire_candidate =
            maneuver_elapsed_ + time_epsilon >= minimum_duration &&
            line_found && line_confidence >= config_.reacquire_min_confidence &&
            feature == RoadFeature::None;
        if (reacquire_candidate) {
            // 一旦新赛道进入视野，就先把控制权渐进交回视觉闭环；只有
            // 转向量也已回到合理范围，才累计稳定重捕获帧。
            target_velocity = tracking_velocity;
            target_omega = tracking_omega;
            if (std::abs(tracking.steering) <=
                config_.reacquire_max_steering) {
                ++reacquire_frames_;
            } else {
                reacquire_frames_ = 0;
            }
        } else {
            reacquire_frames_ = 0;
        }

        if (reacquire_frames_ >= config_.maneuver_reacquire_frames) {
            mode_ = MotionMode::Tracking;
            output.maneuver_completed = true;
            output.action = action_;
            action_.clear();
            maneuver_elapsed_ = 0.0;
            reacquire_frames_ = 0;
        } else if (maneuver_elapsed_ + time_epsilon >= timeout) {
            enterFault("节点动作超时，未重新找到赛道");
        }
    }

    // 普通循线确认丢线停车或任何故障，都绕过斜坡直接停车。
    const bool immediate_stop = mode_ == MotionMode::FaultStop ||
                                (mode_ == MotionMode::Tracking &&
                                 tracking.line_lost &&
                                 tracking.speed <= 0.0);
    if (immediate_stop) {
        previous_velocity_ = 0.0;
        previous_omega_ = 0.0;
    } else {
        previous_velocity_ = limitVelocity(target_velocity, dt_seconds);
        previous_omega_ = limitOmega(target_omega, dt_seconds);
    }

    output.velocity_m_s = previous_velocity_;
    output.omega_rad_s = previous_omega_;
    output.mode = mode_;
    output.action = output.action.empty() ? action_ : output.action;
    output.fault_reason = fault_reason_;
    output.fault_triggered = fault_report_pending_;
    fault_report_pending_ = false;
    return output;
}

void MotionSupervisor::reset() {
    mode_ = MotionMode::Tracking;
    action_.clear();
    fault_reason_.clear();
    maneuver_elapsed_ = 0.0;
    reacquire_frames_ = 0;
    previous_velocity_ = 0.0;
    previous_omega_ = 0.0;
    fault_report_pending_ = false;
    update_initialized_ = false;
}

void MotionSupervisor::enterFault(const std::string& reason) {
    mode_ = MotionMode::FaultStop;
    action_.clear();
    fault_reason_ = reason;
    fault_report_pending_ = true;
}

double MotionSupervisor::limitVelocity(double target,
                                       double dt_seconds) const {
    const double rate = target >= previous_velocity_
                            ? config_.max_accel_m_s2
                            : config_.max_decel_m_s2;
    return approach(previous_velocity_, target, rate * dt_seconds);
}

double MotionSupervisor::limitOmega(double target, double dt_seconds) const {
    return approach(previous_omega_, target,
                    config_.max_omega_accel_rad_s2 * dt_seconds);
}

}  // namespace xunji
