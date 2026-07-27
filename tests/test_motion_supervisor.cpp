#include "motion_supervisor.hpp"

#include <cmath>
#include <iostream>

namespace {

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

xunji::ControlCommand trackingCommand() {
    xunji::ControlCommand command;
    command.speed = 0.4;
    command.steering = -0.5;
    command.line_lost = false;
    return command;
}

}  // namespace

int main() {
    bool ok = true;
    xunji::MotionConfig config;
    config.max_accel_m_s2 = 1.0;
    config.max_decel_m_s2 = 2.0;
    config.max_omega_accel_rad_s2 = 10.0;
    config.turn_duration_seconds = 0.3;
    config.turn_timeout_seconds = 0.8;
    config.straight_node_seconds = 0.2;
    config.straight_timeout_seconds = 0.6;
    config.maneuver_reacquire_frames = 3;

    xunji::MotionSupervisor motion(config);
    auto output = motion.update(trackingCommand(), true, 0.8,
                                xunji::RoadFeature::None, 0.1);
    ok &= check(std::abs(output.velocity_m_s - 0.1) < 1e-9,
                "tracking acceleration should be rate limited");
    ok &= check(output.omega_rad_s > 0.0,
                "negative visual steering should produce positive omega");

    ok &= check(motion.startManeuver("LEFT"),
                "left maneuver should start from tracking");
    xunji::MotionSupervisor overlap_motion(config);
    ok &= check(overlap_motion.startManeuver("LEFT"),
                "overlap test maneuver should start");
    ok &= check(!overlap_motion.startManeuver("RIGHT"),
                "a second maneuver must not replace an active maneuver");
    auto overlap_output = overlap_motion.update(
        trackingCommand(), true, 0.8, xunji::RoadFeature::None, 0.1);
    ok &= check(overlap_output.mode == xunji::MotionMode::FaultStop &&
                    overlap_output.velocity_m_s == 0.0,
                "overlapping node detections must latch a stop");
    for (int frame = 0; frame < 3; ++frame) {
        output = motion.update(trackingCommand(), true, 0.8,
                               xunji::RoadFeature::CornerLeft, 0.1);
    }
    ok &= check(!output.maneuver_completed,
                "minimum duration alone must not complete a turn");
    for (int frame = 0; frame < 2; ++frame) {
        output = motion.update(trackingCommand(), true, 0.8,
                               xunji::RoadFeature::None, 0.1);
        ok &= check(!output.maneuver_completed,
                    "reacquisition must be stable for all configured frames");
    }
    output = motion.update(trackingCommand(), true, 0.8,
                           xunji::RoadFeature::None, 0.1);
    ok &= check(output.maneuver_completed &&
                    output.mode == xunji::MotionMode::Tracking,
                "stable line reacquisition should complete the maneuver");

    xunji::MotionSupervisor timeout_motion(config);
    ok &= check(timeout_motion.startManeuver("RIGHT"),
                "right maneuver should start");
    for (int frame = 0; frame < 8; ++frame) {
        output = timeout_motion.update(trackingCommand(), false, 0.0,
                                       xunji::RoadFeature::None, 0.1);
    }
    ok &= check(output.mode == xunji::MotionMode::FaultStop &&
                    output.velocity_m_s == 0.0 &&
                    output.omega_rad_s == 0.0,
                "maneuver timeout must latch an immediate stop");
    ok &= check(!timeout_motion.startManeuver("LEFT"),
                "fault stop must reject new maneuvers");
    output = timeout_motion.update(trackingCommand(), true, 0.8,
                                   xunji::RoadFeature::None, 0.1);
    ok &= check(output.mode == xunji::MotionMode::FaultStop &&
                    output.velocity_m_s == 0.0,
                "fault stop must remain latched");

    xunji::MotionSupervisor lost_motion(config);
    xunji::ControlCommand lost;
    lost.line_lost = true;
    output = lost_motion.update(lost, false, 0.0,
                                xunji::RoadFeature::None, 0.1);
    ok &= check(output.velocity_m_s == 0.0 && output.omega_rad_s == 0.0,
                "confirmed line loss must bypass rate limits and stop");

    xunji::MotionSupervisor stale_motion(config);
    stale_motion.update(trackingCommand(), true, 0.8,
                        xunji::RoadFeature::None, 0.1);
    output = stale_motion.update(trackingCommand(), true, 0.8,
                                 xunji::RoadFeature::None, 0.25);
    ok &= check(output.mode == xunji::MotionMode::FaultStop &&
                    output.velocity_m_s == 0.0,
                "a stale vision/control cycle must not restart the chassis");

    return ok ? 0 : 1;
}
