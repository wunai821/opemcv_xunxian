#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace xunji {

enum class RoadFeature {
    None,
    CornerLeft,
    CornerRight,
    BranchLeft,
    BranchRight,
    TJunction,
    Crossroad
};

struct PathExits {
    bool top = false;
    bool bottom = false;
    bool left = false;
    bool right = false;
    int count() const;
    int mask() const;  // bit0=下，bit1=上，bit2=左，bit3=右
};

const char* featureName(RoadFeature feature);

struct VisionConfig {
    int process_width = 320;
    int process_height = 240;
    double roi_top_ratio = 0.35;
    double roi_bottom_ratio = 0.98;
    bool dark_line = false;
    int blur_size = 5;
    int morph_size = 3;
    int min_track_width = 8;
    int max_track_width = 300;
    int search_radius = 100;
    int max_missing_rows = 18;
    double center_smoothing = 0.25;
    double probe_inset_ratio = 0.08;
    int probe_min_run = 6;
    int feature_confirm_frames = 3;
    int feature_clear_frames = 6;
};

struct LineResult {
    bool found = false;
    double error = 0.0;       // -1 左，+1 右
    double confidence = 0.0;  // 0..1
    double curvature = 0.0;   // 0..1
    int valid_rows = 0;
    RoadFeature feature = RoadFeature::None;
    PathExits exits;
    bool feature_confirmed = false;  // 仅在新路口确认时置位一帧
    bool corner_found = false;
    cv::Point corner{-1, -1};  // ROI 内坐标
    cv::Rect roi;
    cv::Mat binary;
    std::vector<cv::Point> left;
    std::vector<cv::Point> right;
    std::vector<cv::Point> center;
    std::vector<cv::Point> guide;  // 可包含直角横向段的完整中心轨迹
};

class LineFollower {
public:
    explicit LineFollower(VisionConfig config);
    LineResult process(const cv::Mat& frame);
    cv::Mat drawDebug(const cv::Mat& frame, const LineResult& result) const;

private:
    VisionConfig config_;
    double filtered_error_ = 0.0;
    RoadFeature candidate_feature_ = RoadFeature::None;
    RoadFeature latched_feature_ = RoadFeature::None;
    int candidate_frames_ = 0;
    int clear_frames_ = 0;

    cv::Mat makeBinary(const cv::Mat& roi_bgr) const;
    RoadFeature detectFeature(const cv::Mat& binary, PathExits& exits) const;
    void updateFeatureState(LineResult& result);
    void buildGuide(LineResult& result) const;
};

}  // namespace xunji
