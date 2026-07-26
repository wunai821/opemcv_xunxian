#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace xunji {

Controller::Controller(ControllerConfig config) : config_(config) {}

ControlCommand Controller::update(double error, double confidence,
                                  double curvature, double dt_seconds) {
    ControlCommand command;
    dt_seconds = std::clamp(dt_seconds, 0.001, 0.2);

    if (confidence <= 0.01) {
        lost_time_ += dt_seconds;
        command.line_lost = true;
        command.speed =
            lost_time_ >= config_.lost_stop_seconds ? 0.0 : config_.min_speed;
        command.steering = std::clamp(previous_error_ * config_.kp,
                                      -config_.max_steer, config_.max_steer);
        integral_ = 0.0;
        return command;
    }

    lost_time_ = 0.0;
    command.line_lost = false;
    integral_ = std::clamp(integral_ + error * dt_seconds, -0.5, 0.5);
    const double derivative =
        initialized_ ? (error - previous_error_) / dt_seconds : 0.0;
    initialized_ = true;
    previous_error_ = error;
    command.steering =
        std::clamp(config_.kp * error + config_.ki * integral_ +
                       config_.kd * derivative,
                   -config_.max_steer, config_.max_steer);

    const double slow_down =
        std::clamp(0.65 * std::abs(command.steering) + 0.35 * curvature,
                   0.0, 1.0);
    command.speed = config_.base_speed -
                    (config_.base_speed - config_.min_speed) * slow_down;
    command.speed *= std::clamp(confidence * 1.5, 0.35, 1.0);
    return command;
}

void Controller::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    lost_time_ = 0.0;
    initialized_ = false;
}

}  // namespace xunji
