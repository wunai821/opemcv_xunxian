#include "controller.hpp"
#include "line_follower.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iostream>

namespace {

cv::Mat makeTrack(int offset) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(20, 20, 20));
    std::vector<cv::Point> polygon{
        {260 + offset / 2, 160}, {380 + offset / 2, 160},
        {500 + offset, 479}, {140 + offset, 479}};
    cv::fillConvexPoly(image, polygon, {235, 235, 235});
    return image;
}

cv::Mat makeDarkTrack() {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    std::vector<cv::Point> polygon{
        {260, 160}, {380, 160}, {500, 479}, {140, 479}};
    cv::fillConvexPoly(image, polygon, {20, 20, 20});
    return image;
}

cv::Mat makeFeature(xunji::RoadFeature feature) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    const cv::Scalar black{20, 20, 20};
    const int thickness = 36;
    const cv::Point center{320, 300};
    cv::line(image, center, {320, 479}, black, thickness);
    switch (feature) {
        case xunji::RoadFeature::CornerLeft:
            cv::line(image, center, {0, 300}, black, thickness);
            break;
        case xunji::RoadFeature::CornerRight:
            cv::line(image, center, {639, 300}, black, thickness);
            break;
        case xunji::RoadFeature::BranchLeft:
            cv::line(image, center, {320, 0}, black, thickness);
            cv::line(image, center, {0, 300}, black, thickness);
            break;
        case xunji::RoadFeature::BranchRight:
            cv::line(image, center, {320, 0}, black, thickness);
            cv::line(image, center, {639, 300}, black, thickness);
            break;
        case xunji::RoadFeature::TJunction:
            cv::line(image, {0, 300}, {639, 300}, black, thickness);
            break;
        case xunji::RoadFeature::Crossroad:
            cv::line(image, {320, 0}, {320, 479}, black, thickness);
            cv::line(image, {0, 300}, {639, 300}, black, thickness);
            break;
        case xunji::RoadFeature::None:
            cv::line(image, center, {320, 0}, black, thickness);
            break;
    }
    return image;
}

cv::Mat makePerspectiveCorner(bool turn_left) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    const cv::Scalar black{20, 20, 20};
    const cv::Point bottom =
        turn_left ? cv::Point{430, 479} : cv::Point{210, 479};
    const cv::Point corner =
        turn_left ? cv::Point{340, 300} : cv::Point{300, 300};
    const cv::Point branch =
        turn_left ? cv::Point{0, 240} : cv::Point{639, 240};
    cv::line(image, bottom, corner, black, 36, cv::LINE_AA);
    cv::line(image, corner, branch, black, 36, cv::LINE_AA);
    return image;
}

void saveDebug(const std::string& path, const cv::Mat& source,
               const xunji::LineFollower& follower,
               const xunji::LineResult& result) {
    cv::Mat debug = follower.drawDebug(source, result);
    cv::Mat binary_bgr;
    cv::cvtColor(result.binary, binary_bgr, cv::COLOR_GRAY2BGR);
    cv::copyMakeBorder(binary_bgr, binary_bgr, 0,
                       debug.rows - binary_bgr.rows, 0, 0,
                       cv::BORDER_CONSTANT, cv::Scalar(40, 40, 40));
    cv::Mat combined;
    cv::hconcat(debug, binary_bgr, combined);
    cv::imwrite(path, combined);
}

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

}  // namespace

int main() {
    xunji::VisionConfig config;
    config.center_smoothing = 1.0;
    xunji::LineFollower follower(config);

    const auto center = follower.process(makeTrack(0));
    const auto left = follower.process(makeTrack(-90));
    const auto right = follower.process(makeTrack(90));

    bool ok = true;
    ok &= check(center.found, "center track should be found");
    ok &= check(std::abs(center.error) < 0.08, "center error should be near zero");
    ok &= check(left.found && left.error < -0.12, "left track error should be negative");
    ok &= check(right.found && right.error > 0.12, "right track error should be positive");

    xunji::VisionConfig dark_config;
    dark_config.dark_line = true;
    dark_config.center_smoothing = 1.0;
    xunji::LineFollower dark_follower(dark_config);
    const auto dark = dark_follower.process(makeDarkTrack());
    ok &= check(dark.found && std::abs(dark.error) < 0.08,
                "dark center track should be found");

    for (const bool turn_left : {true, false}) {
        xunji::VisionConfig perspective_config;
        perspective_config.dark_line = true;
        perspective_config.center_smoothing = 1.0;
        xunji::LineFollower perspective_follower(perspective_config);
        const cv::Mat source = makePerspectiveCorner(turn_left);
        const auto result = perspective_follower.process(source);
        const cv::Point expected_corner =
            turn_left ? cv::Point{170, 66} : cv::Point{150, 66};
        ok &= check(result.corner_found,
                    "perspective corner point should be detected");
        if (result.corner_found) {
            ok &= check(cv::norm(result.corner - expected_corner) < 12.0,
                        "perspective corner should be near the drawn vertex");
        }
        saveDebug(turn_left ? "corner_left_debug.png"
                            : "corner_right_debug.png",
                  source, perspective_follower, result);
    }

    for (const auto expected :
         {xunji::RoadFeature::CornerLeft, xunji::RoadFeature::CornerRight,
          xunji::RoadFeature::BranchLeft, xunji::RoadFeature::BranchRight,
          xunji::RoadFeature::TJunction, xunji::RoadFeature::Crossroad}) {
        xunji::VisionConfig feature_config;
        feature_config.dark_line = true;
        feature_config.feature_confirm_frames = 3;
        xunji::LineFollower feature_follower(feature_config);
        xunji::LineResult result;
        for (int frame = 0; frame < 3; ++frame) {
            result = feature_follower.process(makeFeature(expected));
        }
        ok &= check(result.feature == expected,
                    "road feature classification should match");
        ok &= check(result.feature_confirmed,
                    "road feature should confirm after three frames");
        if (expected == xunji::RoadFeature::CornerLeft) {
            ok &= check(result.corner_found,
                        "left corner point should be detected");
            ok &= check(result.guide.size() > 20,
                        "left corner should have a complete guide");
            ok &= check(result.guide.back().x <
                            result.guide.front().x - 80,
                        "left corner guide should include the horizontal arm");
        }
        if (expected == xunji::RoadFeature::CornerRight) {
            ok &= check(result.corner_found,
                        "right corner point should be detected");
            ok &= check(result.guide.size() > 20,
                        "right corner should have a complete guide");
            ok &= check(result.guide.back().x >
                            result.guide.front().x + 80,
                        "right corner guide should include the horizontal arm");
        }
        const auto repeated = feature_follower.process(makeFeature(expected));
        ok &= check(!repeated.feature_confirmed,
                    "latched feature should not emit repeatedly");
        const auto changed =
            feature_follower.process(makeFeature(xunji::RoadFeature::Crossroad));
        ok &= check(!changed.feature_confirmed,
                    "feature changes inside one node should not emit again");
        for (int frame = 0; frame < feature_config.feature_clear_frames; ++frame) {
            feature_follower.process(makeFeature(xunji::RoadFeature::None));
        }
        for (int frame = 0; frame < 3; ++frame) {
            result = feature_follower.process(makeFeature(expected));
        }
        ok &= check(result.feature_confirmed,
                    "feature should emit again after leaving the previous node");
    }

    xunji::Controller controller({});
    const auto command = controller.update(right.error, right.confidence,
                                           right.curvature, 1.0 / 30.0);
    ok &= check(command.steering > 0.0, "right error should steer right");
    ok &= check(command.speed > 0.0, "visible line should allow motion");
    controller.update(0.0, 0.0, 0.0, 0.3);
    controller.update(0.0, 0.0, 0.0, 0.3);
    const auto lost = controller.update(0.0, 0.0, 0.0, 0.3);
    ok &= check(lost.speed == 0.0, "prolonged line loss should stop the car");

    xunji::ControllerConfig derivative_config;
    derivative_config.kp = 0.0;
    derivative_config.ki = 0.0;
    derivative_config.kd = 1.0;
    xunji::Controller derivative_controller(derivative_config);
    derivative_controller.update(0.5, 1.0, 0.0, 1.0 / 30.0);
    derivative_controller.update(0.0, 0.0, 0.0, 1.0 / 30.0);
    const auto reacquired = derivative_controller.update(
        -0.5, 1.0, 0.0, 1.0 / 30.0);
    ok &= check(std::abs(reacquired.steering) < 1e-9,
                "line reacquisition must not create a derivative spike");
    return ok ? 0 : 1;
}
