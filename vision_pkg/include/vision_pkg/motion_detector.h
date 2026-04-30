#ifndef VISION_PKG_MOTION_DETECTOR_H
#define VISION_PKG_MOTION_DETECTOR_H

#include <std_msgs/Header.h>
#include <opencv2/core.hpp>
#include <functional>
#include <mutex>
#include <vector>

namespace vision_pkg
{

class MotionDetector
{
public:
    enum class DebugImage
    {
        Gray,
        FrameDiff,
        ForegroundRaw,
        ForegroundAfterDiff,
        Canny,
        MotionSupport,
        CleanedEdges,
        DepthMask,
        FinalMask
    };

    struct Config
    {
        bool enabled{false};
        bool debug_images{false};
        bool depth_filter{false};
        int min_area{30};
        double canny_low_threshold{50.0};
        double canny_high_threshold{150.0};
        double learning_rate{0.01};
        double diff_threshold{9.0};
        double max_foreground_ratio{0.12};
        double depth_min_m{0.0};
        double depth_max_m{0.7};
        int gaussian_k{5};
        double gaussian_sigma{1.0};
        double gamma{0.75};
        double clahe_clip{2.0};
        int depth_mask_dilate_k{5};
        int depth_mask_dilate_iter{1};
        int roi_dilate_k{9};
        int roi_dilate_iter{6};
        int merge_k{13};
        int merge_iter{3};
        double scene_motion_pct{18.0};
        double min_support_ratio{0.03};
        int min_support_pixels{60};
        int final_thick{2};
        double min_edge_motion_ratio{0.12};
        double box_smoothing_alpha{0.35};
        int box_hold_frames{3};
        int box_padding{6};
    };

    using DebugPublisher =
        std::function<void(DebugImage, const std_msgs::Header&, const cv::Mat&)>;

    MotionDetector(const Config& config, DebugPublisher debug_publisher);

    bool enabled() const;
    void clearCache();
    bool hasCachedResults() const;
    void process(const cv::Mat& src, const cv::Mat& depth, const std_msgs::Header& header);
    void drawCached(cv::Mat& dst) const;

private:
    struct MotionBox
    {
        cv::Rect box;
        double area{0.0};
    };

    struct TrackedMotionBox
    {
        cv::Rect2d box;
        int missed_frames{0};
    };

    static int ensureOdd(int value, int minimum);
    static cv::Mat applyGamma(const cv::Mat& image, double gamma);
    static cv::Mat enhanceGray(const cv::Mat& gray, double gamma, double clahe_clip);
    static cv::Mat buildColorCanny(const cv::Mat& bgr,
                                   const cv::Mat& enhanced_gray,
                                   double low_threshold,
                                   double high_threshold);
    static double computeMotionSpread(const cv::Mat& mask, int grid_rows = 4, int grid_cols = 4);
    static cv::Mat expandConnectedEdges(const cv::Mat& masked_canny,
                                        const cv::Mat& seed_mask,
                                        double min_edge_motion_ratio);
    static cv::Mat mergeMotionTargets(const cv::Mat& motion_mask,
                                      int merge_kernel_size,
                                      int merge_iterations);
    static cv::Mat suppressInnerEdgesForNearClosedTargets(const cv::Mat& source_edge_mask,
                                                          const cv::Mat& merged_mask,
                                                          double min_area);
    static cv::Mat removeSmallComponents(const cv::Mat& mask, double min_area);
    static cv::Mat closeAndFillMotionForeground(const cv::Mat& mask,
                                                int close_kernel_size,
                                                int close_iterations,
                                                double min_area);

    cv::Mat filterTargetsByMotionSupport(const cv::Mat& target_mask,
                                         const cv::Mat& motion_support_mask,
                                         double min_area) const;
    static std::vector<MotionBox> mergeNearbyBoxes(const std::vector<MotionBox>& boxes,
                                                   const cv::Size& image_size);
    static cv::Rect padBox(const cv::Rect& box, int padding, const cv::Size& image_size);
    static double rectIou(const cv::Rect2d& a, const cv::Rect2d& b);
    static double boxCenterDistance(const cv::Rect2d& a, const cv::Rect2d& b);
    static bool boxesCanMatch(const cv::Rect2d& a, const cv::Rect2d& b);
    static cv::Rect2d smoothBox(const cv::Rect2d& previous,
                                const cv::Rect2d& detected,
                                double alpha);
    static cv::Rect2d clipBox(const cv::Rect2d& box, const cv::Size& image_size);
    std::vector<MotionBox> smoothTrackedBoxes(const std::vector<MotionBox>& boxes,
                                              const cv::Size& image_size);
    cv::Mat buildDepthMask(const cv::Mat& depth, const cv::Size& output_size) const;
    void publishDebug(DebugImage kind, const std_msgs::Header& header, const cv::Mat& image) const;

    Config config_;
    DebugPublisher debug_publisher_;

    cv::Mat prev_motion_gray_frame_;
    cv::Mat prev_scene_gray_frame_;
    cv::Mat prev_foreground_mask_;
    std::vector<TrackedMotionBox> tracked_boxes_;

    mutable std::mutex result_mutex_;
    std::vector<MotionBox> last_boxes_;
    cv::Mat last_motion_mask_;
    cv::Size last_image_size_;
};

}  // namespace vision_pkg

#endif  // VISION_PKG_MOTION_DETECTOR_H
