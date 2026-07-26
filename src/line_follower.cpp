#include "line_follower.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>

namespace xunji {
namespace {

int longestRunOnRow(const cv::Mat& image, int y, int x1, int x2) {
    int longest = 0;
    int current = 0;
    const auto* row = image.ptr<unsigned char>(y);
    for (int x = x1; x <= x2; ++x) {
        if (row[x] != 0) {
            longest = std::max(longest, ++current);
        } else {
            current = 0;
        }
    }
    return longest;
}

int longestRunOnColumn(const cv::Mat& image, int x, int y1, int y2) {
    int longest = 0;
    int current = 0;
    for (int y = y1; y <= y2; ++y) {
        if (image.ptr<unsigned char>(y)[x] != 0) {
            longest = std::max(longest, ++current);
        } else {
            current = 0;
        }
    }
    return longest;
}

}  // namespace

int PathExits::count() const {
    return static_cast<int>(top) + static_cast<int>(bottom) +
           static_cast<int>(left) + static_cast<int>(right);
}

int PathExits::mask() const {
    return (bottom ? 1 : 0) | (top ? 2 : 0) | (left ? 4 : 0) |
           (right ? 8 : 0);
}

const char* featureName(RoadFeature feature) {
    switch (feature) {
        case RoadFeature::CornerLeft: return "CORNER_LEFT";
        case RoadFeature::CornerRight: return "CORNER_RIGHT";
        case RoadFeature::BranchLeft: return "BRANCH_LEFT";
        case RoadFeature::BranchRight: return "BRANCH_RIGHT";
        case RoadFeature::TJunction: return "T_JUNCTION";
        case RoadFeature::Crossroad: return "CROSSROAD";
        case RoadFeature::None: return "NONE";
    }
    return "NONE";
}

LineFollower::LineFollower(VisionConfig config) : config_(config) {
    if (config_.process_width < 32 || config_.process_height < 32) {
        throw std::invalid_argument("process image is too small");
    }
    config_.blur_size = std::max(1, config_.blur_size | 1);
    config_.morph_size = std::max(1, config_.morph_size | 1);
}

cv::Mat LineFollower::makeBinary(const cv::Mat& roi_bgr) const {
    cv::Mat gray;
    cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, {config_.blur_size, config_.blur_size}, 0.0);

    cv::Mat binary(gray.size(), CV_8UC1);
    const int split1 = gray.rows / 3;
    const int split2 = gray.rows * 2 / 3;
    const int threshold_type =
        (config_.dark_line ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY) |
        cv::THRESH_OTSU;

    // 与参考项目一致：远、中、近三区分别计算 Otsu，适应赛道纵向光照差异。
    for (const auto& range : {std::pair<int, int>{0, split1},
                              {split1, split2},
                              {split2, gray.rows}}) {
        cv::Mat src_part = gray.rowRange(range.first, range.second);
        cv::Mat dst_part = binary.rowRange(range.first, range.second);
        cv::threshold(src_part, dst_part, 0, 255, threshold_type);
    }

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, {config_.morph_size, config_.morph_size});
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    return binary;
}

RoadFeature LineFollower::detectFeature(const cv::Mat& binary,
                                        PathExits& exits) const {
    const int inset_x = std::clamp(
        static_cast<int>(binary.cols * config_.probe_inset_ratio),
        2, binary.cols / 3);
    const int inset_y = std::clamp(
        static_cast<int>(binary.rows * config_.probe_inset_ratio),
        2, binary.rows / 3);
    const int left_x = inset_x;
    const int right_x = binary.cols - 1 - inset_x;
    const int top_y = inset_y;
    const int bottom_y = binary.rows - 1 - inset_y;

    exits.top = longestRunOnRow(binary, top_y, left_x, right_x) >=
                config_.probe_min_run;
    exits.bottom = longestRunOnRow(binary, bottom_y, left_x, right_x) >=
                   config_.probe_min_run;
    exits.left = longestRunOnColumn(binary, left_x, top_y, bottom_y) >=
                 config_.probe_min_run;
    exits.right = longestRunOnColumn(binary, right_x, top_y, bottom_y) >=
                  config_.probe_min_run;

    // 下方出口代表车辆来路。没有来路时不把孤立噪声判断为路口。
    if (!exits.bottom) {
        return RoadFeature::None;
    }
    if (exits.top && exits.left && exits.right) {
        return RoadFeature::Crossroad;
    }
    if (!exits.top && exits.left && exits.right) {
        return RoadFeature::TJunction;
    }
    if (exits.top && exits.left && !exits.right) {
        return RoadFeature::BranchLeft;
    }
    if (exits.top && exits.right && !exits.left) {
        return RoadFeature::BranchRight;
    }
    if (!exits.top && exits.left && !exits.right) {
        return RoadFeature::CornerLeft;
    }
    if (!exits.top && exits.right && !exits.left) {
        return RoadFeature::CornerRight;
    }
    return RoadFeature::None;
}

void LineFollower::updateFeatureState(LineResult& result) {
    if (result.feature == RoadFeature::None) {
        candidate_feature_ = RoadFeature::None;
        candidate_frames_ = 0;
        if (latched_feature_ != RoadFeature::None &&
            ++clear_frames_ >= config_.feature_clear_frames) {
            latched_feature_ = RoadFeature::None;
            clear_frames_ = 0;
        }
        return;
    }

    clear_frames_ = 0;
    // 同一路口内分类可能随视角从 CROSSROAD 短暂变成 T_JUNCTION。
    // 必须等探测框连续清空后才允许发送下一个节点。
    if (latched_feature_ != RoadFeature::None) {
        return;
    }
    if (result.feature != candidate_feature_) {
        candidate_feature_ = result.feature;
        candidate_frames_ = 1;
        if (config_.feature_confirm_frames <= 1) {
            latched_feature_ = result.feature;
            candidate_feature_ = RoadFeature::None;
            candidate_frames_ = 0;
            result.feature_confirmed = true;
        }
        return;
    }
    if (++candidate_frames_ >= config_.feature_confirm_frames) {
        latched_feature_ = result.feature;
        candidate_feature_ = RoadFeature::None;
        candidate_frames_ = 0;
        result.feature_confirmed = true;
    }
}

void LineFollower::buildGuide(LineResult& result) const {
    result.guide = result.center;
    const bool turn_left = result.feature == RoadFeature::CornerLeft;
    const bool turn_right = result.feature == RoadFeature::CornerRight;
    if ((!turn_left && !turn_right) || result.center.size() < 16) {
        return;
    }

    // 底部若干行只包含来路，用中位数估计正常线宽。
    const int near_count = std::min<int>(24, result.center.size() / 2);
    std::vector<int> near_width;
    near_width.reserve(near_count);
    for (int i = 0; i < near_count; ++i) {
        near_width.push_back(result.right[i].x - result.left[i].x + 1);
    }
    const auto median = [](std::vector<int>& values) {
        const auto middle = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), middle, values.end());
        return *middle;
    };
    const int incoming_width = std::max(1, median(near_width));

    // 到达横向线段后，逐行找到的白色连续段宽度会明显增大。
    // 直接使用每一行实际边界，不再假设来路中心 x 始终不变。
    int widest = 0;
    for (std::size_t i = 0; i < result.center.size(); ++i) {
        widest = std::max(widest,
                          result.right[i].x - result.left[i].x + 1);
    }
    if (widest < incoming_width * 2) {
        return;
    }

    std::vector<int> junction_rows;
    const int wide_threshold =
        std::max(incoming_width * 2, widest * 2 / 3);
    for (std::size_t i = 0; i < result.center.size(); ++i) {
        const int width = result.right[i].x - result.left[i].x + 1;
        if (width >= std::max(incoming_width * 2, widest * 2 / 3)) {
            junction_rows.push_back(result.center[i].y);
        }
    }
    if (junction_rows.empty()) {
        return;
    }
    const int junction_y =
        std::accumulate(junction_rows.begin(), junction_rows.end(), 0) /
        static_cast<int>(junction_rows.size());

    // 第一组点拟合进入直角的中心线。排除横线造成的宽行和交汇处圆角。
    std::vector<cv::Point2f> incoming_points;
    for (std::size_t i = 0; i < result.center.size(); ++i) {
        const int width = result.right[i].x - result.left[i].x + 1;
        if (result.center[i].y > junction_y + incoming_width &&
            width < wide_threshold) {
            incoming_points.emplace_back(result.center[i]);
        }
    }
    if (incoming_points.size() < 6) {
        return;
    }

    cv::Vec4f incoming_fit;
    cv::fitLine(incoming_points, incoming_fit, cv::DIST_L2, 0, 0.01, 0.01);
    const cv::Point2f incoming_dir{incoming_fit[0], incoming_fit[1]};
    const cv::Point2f incoming_origin{incoming_fit[2], incoming_fit[3]};
    if (std::abs(incoming_dir.y) < 0.15f) {
        return;
    }
    const float estimated_corner_x =
        incoming_origin.x +
        (junction_y - incoming_origin.y) * incoming_dir.x / incoming_dir.y;

    // 第二组点逐列提取转弯后的中心。跳过交汇圆角，防止来路线段拉偏横线拟合。
    const int direction = turn_left ? -1 : 1;
    const int start_x = std::clamp(
        static_cast<int>(std::lround(estimated_corner_x)) +
            direction * incoming_width * 2,
        0, result.binary.cols - 1);
    std::vector<cv::Point2f> branch_points;
    float predicted_y = static_cast<float>(junction_y);
    int misses = 0;
    for (int x = start_x;
         x >= 0 && x < result.binary.cols; x += direction * 2) {
        int seed_y = -1;
        const int search_y = std::clamp(
            static_cast<int>(std::lround(predicted_y)), 0,
            result.binary.rows - 1);
        for (int d = 0; d <= std::max(24, incoming_width * 3); ++d) {
            const int upper = search_y - d;
            const int lower = search_y + d;
            if (upper >= 0 &&
                result.binary.ptr<unsigned char>(upper)[x] != 0) {
                seed_y = upper;
                break;
            }
            if (lower < result.binary.rows &&
                result.binary.ptr<unsigned char>(lower)[x] != 0) {
                seed_y = lower;
                break;
            }
        }
        if (seed_y < 0) {
            if (++misses > 3) break;
            continue;
        }
        misses = 0;
        int upper = seed_y;
        int lower = seed_y;
        while (upper > 0 &&
               result.binary.ptr<unsigned char>(upper - 1)[x] != 0) {
            --upper;
        }
        while (lower + 1 < result.binary.rows &&
               result.binary.ptr<unsigned char>(lower + 1)[x] != 0) {
            ++lower;
        }
        const int vertical_width = lower - upper + 1;
        if (vertical_width > incoming_width * 4) {
            continue;
        }
        predicted_y = (upper + lower) * 0.5f;
        branch_points.emplace_back(static_cast<float>(x), predicted_y);
    }
    if (branch_points.size() < 8) {
        return;
    }

    cv::Vec4f branch_fit;
    cv::fitLine(branch_points, branch_fit, cv::DIST_L2, 0, 0.01, 0.01);
    const cv::Point2f branch_dir{branch_fit[0], branch_fit[1]};
    const cv::Point2f branch_origin{branch_fit[2], branch_fit[3]};

    // 两条拟合中心线的交点才是真正角点；它不受黑线厚度和透视倾斜影响。
    const float cross =
        incoming_dir.x * branch_dir.y - incoming_dir.y * branch_dir.x;
    if (std::abs(cross) < 0.35f) {
        return;
    }
    const cv::Point2f delta = branch_origin - incoming_origin;
    const float t =
        (delta.x * branch_dir.y - delta.y * branch_dir.x) / cross;
    const cv::Point2f corner = incoming_origin + incoming_dir * t;
    if (corner.x < -incoming_width ||
        corner.x >= result.binary.cols + incoming_width ||
        corner.y < -incoming_width ||
        corner.y >= result.binary.rows + incoming_width) {
        return;
    }
    result.corner = {
        std::clamp(static_cast<int>(std::lround(corner.x)), 0,
                   result.binary.cols - 1),
        std::clamp(static_cast<int>(std::lround(corner.y)), 0,
                   result.binary.rows - 1)};
    result.corner_found = true;

    const auto project = [](const cv::Point2f& point,
                            const cv::Point2f& origin,
                            const cv::Point2f& direction_vector) {
        return origin + direction_vector *
                            ((point - origin).dot(direction_vector));
    };
    const cv::Point2f incoming_end =
        project(incoming_points.front(), incoming_origin, incoming_dir);
    const cv::Point2f branch_end =
        project(branch_points.back(), branch_origin, branch_dir);

    std::vector<cv::Point> fitted_guide;
    const auto appendSegment = [&fitted_guide](const cv::Point2f& from,
                                               const cv::Point2f& to) {
        const float length = cv::norm(to - from);
        const int steps = std::max(1, static_cast<int>(length / 2.0f));
        for (int i = fitted_guide.empty() ? 0 : 1; i <= steps; ++i) {
            const float ratio = static_cast<float>(i) / steps;
            const cv::Point2f point = from + (to - from) * ratio;
            fitted_guide.emplace_back(
                static_cast<int>(std::lround(point.x)),
                static_cast<int>(std::lround(point.y)));
        }
    };
    appendSegment(incoming_end, corner);
    appendSegment(corner, branch_end);
    if (fitted_guide.size() > 12) {
        result.guide = std::move(fitted_guide);
    }
}

LineResult LineFollower::process(const cv::Mat& frame) {
    LineResult result;
    if (frame.empty()) {
        return result;
    }

    cv::Mat resized;
    cv::resize(frame, resized, {config_.process_width, config_.process_height});
    const int top = std::clamp(
        static_cast<int>(std::lround(config_.roi_top_ratio * resized.rows)),
        0, resized.rows - 2);
    const int bottom = std::clamp(
        static_cast<int>(std::lround(config_.roi_bottom_ratio * resized.rows)),
        top + 1, resized.rows);
    result.roi = {0, top, resized.cols, bottom - top};
    result.binary = makeBinary(resized(result.roi));
    result.feature = detectFeature(result.binary, result.exits);
    updateFeatureState(result);

    int predicted_center = result.binary.cols / 2;
    int missing_rows = 0;
    for (int y = result.binary.rows - 1; y >= 0; --y) {
        const auto* row = result.binary.ptr<unsigned char>(y);
        int seed = -1;
        for (int d = 0; d <= config_.search_radius; ++d) {
            const int lx = predicted_center - d;
            const int rx = predicted_center + d;
            if (lx >= 0 && row[lx] != 0) {
                seed = lx;
                break;
            }
            if (rx < result.binary.cols && row[rx] != 0) {
                seed = rx;
                break;
            }
        }

        if (seed < 0) {
            if (++missing_rows > config_.max_missing_rows) {
                break;
            }
            continue;
        }

        int left = seed;
        int right = seed;
        while (left > 0 && row[left - 1] != 0) {
            --left;
        }
        while (right + 1 < result.binary.cols && row[right + 1] != 0) {
            ++right;
        }
        const int width = right - left + 1;
        if (width < config_.min_track_width || width > config_.max_track_width) {
            if (++missing_rows > config_.max_missing_rows) {
                break;
            }
            continue;
        }

        missing_rows = 0;
        predicted_center = (left + right) / 2;
        result.left.emplace_back(left, y);
        result.right.emplace_back(right, y);
        result.center.emplace_back(predicted_center, y);
    }

    result.valid_rows = static_cast<int>(result.center.size());
    buildGuide(result);
    const int minimum_rows = std::max(8, result.binary.rows / 6);
    if (result.valid_rows < minimum_rows) {
        return result;
    }

    double weighted_error = 0.0;
    double weight_sum = 0.0;
    for (const auto& point : result.center) {
        const double near_weight =
            0.25 + 0.75 * point.y / std::max(1, result.binary.rows - 1);
        weighted_error +=
            near_weight * (point.x - result.binary.cols * 0.5);
        weight_sum += near_weight;
    }
    const double raw_error =
        weighted_error / weight_sum / (result.binary.cols * 0.5);
    filtered_error_ = config_.center_smoothing * raw_error +
                      (1.0 - config_.center_smoothing) * filtered_error_;
    result.error = std::clamp(filtered_error_, -1.0, 1.0);
    result.confidence = std::clamp(
        static_cast<double>(result.valid_rows) / result.binary.rows, 0.0, 1.0);

    const int sample = std::min(12, result.valid_rows / 3);
    if (sample > 0) {
        const double near_x = result.center[sample - 1].x;
        const double far_x = result.center[result.valid_rows - sample].x;
        result.curvature =
            std::clamp(std::abs(far_x - near_x) / (result.binary.cols * 0.35),
                       0.0, 1.0);
    }
    result.found = true;
    return result;
}

cv::Mat LineFollower::drawDebug(const cv::Mat& frame,
                                const LineResult& result) const {
    cv::Mat output;
    cv::resize(frame, output, {config_.process_width, config_.process_height});
    cv::rectangle(output, result.roi, {255, 100, 0}, 1);
    const int inset_x = static_cast<int>(
        result.binary.cols * config_.probe_inset_ratio);
    const int inset_y = static_cast<int>(
        result.binary.rows * config_.probe_inset_ratio);
    cv::rectangle(output,
                  {inset_x, result.roi.y + inset_y,
                   result.binary.cols - inset_x * 2,
                   result.binary.rows - inset_y * 2},
                  {255, 0, 255}, 1);
    for (std::size_t i = 0; i < result.center.size(); ++i) {
        const cv::Point offset{0, result.roi.y};
        cv::circle(output, result.left[i] + offset, 1, {0, 0, 255}, -1);
        cv::circle(output, result.right[i] + offset, 1, {255, 0, 0}, -1);
    }
    if (result.guide.size() >= 2) {
        std::vector<cv::Point> guide = result.guide;
        for (auto& point : guide) {
            point.y += result.roi.y;
        }
        cv::polylines(output, guide, false, {0, 255, 0}, 2, cv::LINE_AA);
    }
    if (result.corner_found) {
        const cv::Point corner_on_frame =
            result.corner + cv::Point{0, result.roi.y};
        cv::circle(output, corner_on_frame, 7, {0, 165, 255}, -1,
                   cv::LINE_AA);
        cv::circle(output, corner_on_frame, 10, {255, 255, 255}, 2,
                   cv::LINE_AA);
        cv::putText(output,
                    "corner(" + std::to_string(result.corner.x) + "," +
                        std::to_string(result.corner.y) + ")",
                    corner_on_frame + cv::Point{10, -10},
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, {0, 165, 255}, 1,
                    cv::LINE_AA);
    }
    const int target_x = static_cast<int>(
        output.cols * 0.5 * (1.0 + result.error));
    cv::line(output, {output.cols / 2, output.rows - 1},
             {target_x, result.roi.y}, {0, 255, 255}, 2);
    cv::putText(output, featureName(result.feature), {8, 22},
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                result.feature_confirmed ? cv::Scalar(0, 255, 255)
                                         : cv::Scalar(255, 255, 255),
                2);
    return output;
}

}  // namespace xunji
