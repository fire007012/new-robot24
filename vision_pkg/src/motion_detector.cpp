#include "vision_pkg/motion_detector.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace vision_pkg
{

MotionDetector::MotionDetector(const Config& config, DebugPublisher debug_publisher)
    : config_(config), debug_publisher_(std::move(debug_publisher))
{
    config_.gaussian_k = ensureOdd(config_.gaussian_k, 1);
    config_.median_k = ensureOdd(config_.median_k, 1);
    config_.depth_mask_dilate_k = ensureOdd(config_.depth_mask_dilate_k, 1);
    config_.roi_dilate_k = ensureOdd(config_.roi_dilate_k, 1);
    config_.merge_k = ensureOdd(config_.merge_k, 3);
    config_.final_thick = std::max(1, config_.final_thick);
    config_.learning_rate = std::max(0.0, std::min(1.0, config_.learning_rate));
    config_.max_foreground_ratio = std::max(0.0, config_.max_foreground_ratio);
    config_.min_edge_motion_ratio = std::max(0.0, config_.min_edge_motion_ratio);
    config_.box_smoothing_alpha = std::max(0.0, std::min(1.0, config_.box_smoothing_alpha));
    config_.box_hold_frames = std::max(0, config_.box_hold_frames);
    config_.box_padding = std::max(0, config_.box_padding);
}

bool MotionDetector::enabled() const
{
    return config_.enabled;
}

bool MotionDetector::hasCachedResults() const
{
    std::lock_guard<std::mutex> lock(result_mutex_);
    return !last_motion_mask_.empty() && cv::countNonZero(last_motion_mask_) > 0;
}

void MotionDetector::clearCache()
{
    std::lock_guard<std::mutex> lock(result_mutex_);
    last_boxes_.clear();
    last_motion_mask_.release();
    last_image_size_ = cv::Size();
}

void MotionDetector::publishDebug(DebugImage kind,
                                  const std_msgs::Header& header,
                                  const cv::Mat& image) const
{
    if (!config_.debug_images || image.empty() || !debug_publisher_) {
        return;
    }
    debug_publisher_(kind, header, image);
}

int MotionDetector::ensureOdd(int value, int minimum)
{
    value = std::max(value, minimum);
    if (value % 2 == 0) {
        ++value;
    }
    return value;
}

cv::Mat MotionDetector::applyGamma(const cv::Mat& image, double gamma)
{
    gamma = std::max(0.1, gamma);
    if (std::abs(gamma - 1.0) < 1e-6) {
        return image.clone();
    }

    cv::Mat table(1, 256, CV_8UC1);
    uint8_t* row = table.ptr<uint8_t>(0);
    for (int i = 0; i < 256; ++i) {
        row[i] = cv::saturate_cast<uint8_t>(std::pow(i / 255.0, gamma) * 255.0);
    }

    cv::Mat adjusted;
    cv::LUT(image, table, adjusted);
    return adjusted;
}

cv::Mat MotionDetector::enhanceGray(const cv::Mat& gray, double gamma, double clahe_clip)
{
    cv::Mat enhanced = applyGamma(gray, gamma);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(std::max(0.1, clahe_clip), cv::Size(8, 8));
    cv::Mat result;
    clahe->apply(enhanced, result);
    return result;
}

cv::Mat MotionDetector::buildColorCanny(const cv::Mat& bgr,
                                        const cv::Mat& enhanced_gray,
                                        double low_threshold,
                                        double high_threshold)
{
    cv::Mat color_edges;
    cv::Canny(enhanced_gray, color_edges, low_threshold, high_threshold);
    if (bgr.empty() || bgr.channels() != 3 || bgr.size() != enhanced_gray.size()) {
        return color_edges;
    }

    std::vector<cv::Mat> channels;
    cv::split(bgr, channels);
    for (const auto& channel : channels) {
        cv::Mat channel_edges;
        cv::Canny(channel, channel_edges, low_threshold, high_threshold);
        cv::bitwise_or(color_edges, channel_edges, color_edges);
    }
    return color_edges;
}

double MotionDetector::computeMotionSpread(const cv::Mat& mask, int grid_rows, int grid_cols)
{
    if (mask.empty() || cv::countNonZero(mask) == 0) {
        return 0.0;
    }

    int active_cells = 0;
    const int total_cells = grid_rows * grid_cols;
    for (int row = 0; row < grid_rows; ++row) {
        const int y0 = row * mask.rows / grid_rows;
        const int y1 = (row + 1) * mask.rows / grid_rows;
        for (int col = 0; col < grid_cols; ++col) {
            const int x0 = col * mask.cols / grid_cols;
            const int x1 = (col + 1) * mask.cols / grid_cols;
            const cv::Mat cell = mask(cv::Rect(x0, y0, std::max(1, x1 - x0), std::max(1, y1 - y0)));
            const int min_pixels = std::max(20, static_cast<int>(cell.total() * 0.01));
            if (cv::countNonZero(cell) >= min_pixels) {
                ++active_cells;
            }
        }
    }

    return static_cast<double>(active_cells) / static_cast<double>(total_cells);
}

cv::Mat MotionDetector::expandConnectedEdges(const cv::Mat& masked_canny,
                                             const cv::Mat& seed_mask,
                                             double min_edge_motion_ratio)
{
    if (masked_canny.empty() || seed_mask.empty() ||
        cv::countNonZero(masked_canny) == 0 || cv::countNonZero(seed_mask) == 0) {
        return cv::Mat::zeros(masked_canny.size(), CV_8UC1);
    }

    cv::Mat binary_map;
    cv::threshold(masked_canny, binary_map, 0, 1, cv::THRESH_BINARY);

    cv::Mat labels;
    const int num_labels = cv::connectedComponents(binary_map, labels, 8, CV_32S);
    if (num_labels <= 1) {
        return cv::Mat::zeros(masked_canny.size(), CV_8UC1);
    }

    std::vector<int> label_pixels(static_cast<size_t>(num_labels), 0);
    std::vector<int> seed_pixels(static_cast<size_t>(num_labels), 0);
    for (int y = 0; y < labels.rows; ++y) {
        const int* label_row = labels.ptr<int>(y);
        const uint8_t* seed_row = seed_mask.ptr<uint8_t>(y);
        for (int x = 0; x < labels.cols; ++x) {
            const int label = label_row[x];
            if (label <= 0 || label >= num_labels) continue;
            ++label_pixels[static_cast<size_t>(label)];
            if (seed_row[x] != 0) {
                ++seed_pixels[static_cast<size_t>(label)];
            }
        }
    }

    std::vector<uint8_t> selected(static_cast<size_t>(num_labels), 0);
    for (int label = 1; label < num_labels; ++label) {
        const int total = label_pixels[static_cast<size_t>(label)];
        const int seed = seed_pixels[static_cast<size_t>(label)];
        if (total <= 0 || seed <= 0) continue;
        const double ratio = static_cast<double>(seed) / static_cast<double>(total);
        if (ratio >= min_edge_motion_ratio) {
            selected[static_cast<size_t>(label)] = 1;
        }
    }

    cv::Mat expanded_mask(masked_canny.size(), CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < labels.rows; ++y) {
        const int* label_row = labels.ptr<int>(y);
        uint8_t* out_row = expanded_mask.ptr<uint8_t>(y);
        for (int x = 0; x < labels.cols; ++x) {
            const int label = label_row[x];
            if (label > 0 && label < num_labels && selected[static_cast<size_t>(label)] != 0) {
                out_row[x] = 255;
            }
        }
    }

    cv::Mat result;
    cv::bitwise_and(masked_canny, expanded_mask, result);
    return result;
}

cv::Mat MotionDetector::mergeMotionTargets(const cv::Mat& motion_mask,
                                           int merge_kernel_size,
                                           int merge_iterations)
{
    if (motion_mask.empty() || cv::countNonZero(motion_mask) == 0) {
        return cv::Mat::zeros(motion_mask.size(), CV_8UC1);
    }

    const int merge_k = ensureOdd(std::max(3, merge_kernel_size), 3);
    const int merge_iter = std::max(0, merge_iterations);
    if (merge_iter <= 0) {
        return motion_mask.clone();
    }
    const cv::Mat merge_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(merge_k, merge_k));

    cv::Mat merged_mask;
    cv::dilate(motion_mask, merged_mask, merge_kernel, cv::Point(-1, -1), merge_iter);
    cv::morphologyEx(merged_mask, merged_mask, cv::MORPH_CLOSE, merge_kernel);
    cv::erode(merged_mask, merged_mask, merge_kernel, cv::Point(-1, -1), merge_iter);

    if (cv::countNonZero(merged_mask) == 0) {
        return motion_mask.clone();
    }
    return merged_mask;
}

cv::Mat MotionDetector::suppressInnerEdgesForNearClosedTargets(const cv::Mat& source_edge_mask,
                                                               const cv::Mat& merged_mask,
                                                               double min_area)
{
    if (source_edge_mask.empty() || merged_mask.empty() ||
        cv::countNonZero(source_edge_mask) == 0 || cv::countNonZero(merged_mask) == 0) {
        return source_edge_mask.clone();
    }

    cv::Mat cleaned_mask = source_edge_mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(merged_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const cv::Mat inner_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    for (const auto& contour : contours) {
        const double contour_area = cv::contourArea(contour);
        if (contour_area < min_area) continue;

        cv::Mat fill_mask(merged_mask.size(), CV_8UC1, cv::Scalar(0));
        cv::drawContours(fill_mask, std::vector<std::vector<cv::Point>>(1, contour), -1,
                         cv::Scalar(255), cv::FILLED);

        cv::Mat masked_edges;
        cv::bitwise_and(source_edge_mask, fill_mask, masked_edges);
        const int edge_pixels = cv::countNonZero(masked_edges);
        const int fill_pixels = cv::countNonZero(fill_mask);
        if (edge_pixels <= 0) continue;

        const double closure_ratio = static_cast<double>(fill_pixels) / edge_pixels;
        if (closure_ratio < 2.5) continue;

        cv::Mat inner_mask;
        cv::erode(fill_mask, inner_mask, inner_kernel);
        cleaned_mask.setTo(0, inner_mask);
        cv::drawContours(cleaned_mask, std::vector<std::vector<cv::Point>>(1, contour), -1,
                         cv::Scalar(255), 2);
    }

    return cleaned_mask;
}

cv::Mat MotionDetector::filterTargetsByMotionSupport(const cv::Mat& target_mask,
                                                     const cv::Mat& motion_support_mask,
                                                     double min_area) const
{
    if (target_mask.empty() || motion_support_mask.empty() ||
        cv::countNonZero(target_mask) == 0 || cv::countNonZero(motion_support_mask) == 0) {
        return cv::Mat::zeros(target_mask.size(), CV_8UC1);
    }

    cv::Mat filtered_mask(target_mask.size(), CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(target_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        const double contour_area = cv::contourArea(contour);
        if (contour_area < min_area) continue;

        cv::Mat fill_mask(target_mask.size(), CV_8UC1, cv::Scalar(0));
        cv::drawContours(fill_mask, std::vector<std::vector<cv::Point>>(1, contour), -1,
                         cv::Scalar(255), cv::FILLED);
        const int fill_pixels = cv::countNonZero(fill_mask);
        if (fill_pixels <= 0) continue;

        cv::Mat supported_motion;
        cv::bitwise_and(motion_support_mask, fill_mask, supported_motion);
        const int support_pixels = cv::countNonZero(supported_motion);
        const double support_ratio = static_cast<double>(support_pixels) / fill_pixels;
        const int min_support_pixels =
            std::max(config_.min_support_pixels, static_cast<int>(min_area * 0.35));

        if (support_pixels < min_support_pixels || support_ratio < config_.min_support_ratio) {
            continue;
        }

        cv::drawContours(filtered_mask, std::vector<std::vector<cv::Point>>(1, contour), -1,
                         cv::Scalar(255), cv::FILLED);
    }

    cv::Mat result;
    cv::bitwise_and(target_mask, filtered_mask, result);
    return result;
}

cv::Mat MotionDetector::removeSmallComponents(const cv::Mat& mask, double min_area)
{
    if (mask.empty() || cv::countNonZero(mask) == 0) {
        return cv::Mat::zeros(mask.size(), CV_8UC1);
    }

    cv::Mat filtered(mask.size(), CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < min_area) continue;
        cv::drawContours(filtered, std::vector<std::vector<cv::Point>>(1, contour), -1,
                         cv::Scalar(255), cv::FILLED);
    }
    return filtered;
}

cv::Mat MotionDetector::closeAndFillMotionForeground(const cv::Mat& mask,
                                                     int close_kernel_size,
                                                     int close_iterations,
                                                     double min_area)
{
    if (mask.empty() || cv::countNonZero(mask) == 0) {
        return cv::Mat::zeros(mask.size(), CV_8UC1);
    }

    const int close_k = ensureOdd(std::max(3, close_kernel_size), 3);
    const int close_iter = std::max(1, close_iterations);
    const cv::Mat close_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(close_k, close_k));

    cv::Mat closed;
    cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, close_kernel,
                     cv::Point(-1, -1), close_iter);
    cv::dilate(closed, closed, close_kernel, cv::Point(-1, -1), 1);

    cv::Mat filled(closed.size(), CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < min_area) continue;
        cv::drawContours(filled, std::vector<std::vector<cv::Point>>(1, contour), -1,
                         cv::Scalar(255), cv::FILLED);
    }

    return removeSmallComponents(filled, min_area);
}

std::vector<MotionDetector::MotionBox> MotionDetector::mergeNearbyBoxes(
    const std::vector<MotionBox>& boxes,
    const cv::Size& image_size)
{
    if (boxes.size() <= 1) {
        return boxes;
    }

    const int merge_gap_x = std::max(18, image_size.width / 45);
    const int merge_gap_y = std::max(45, image_size.height / 7);
    std::vector<MotionBox> merged = boxes;
    bool changed = true;

    while (changed) {
        changed = false;
        std::vector<MotionBox> next_boxes;
        std::vector<uint8_t> used(merged.size(), 0);

        for (size_t i = 0; i < merged.size(); ++i) {
            if (used[i] != 0) continue;

            cv::Rect current = merged[i].box;
            double area = merged[i].area;
            used[i] = 1;

            for (size_t j = i + 1; j < merged.size(); ++j) {
                if (used[j] != 0) continue;

                const cv::Rect other = merged[j].box;
                const int overlap_x =
                    std::min(current.x + current.width, other.x + other.width) -
                    std::max(current.x, other.x);
                const int overlap_y =
                    std::min(current.y + current.height, other.y + other.height) -
                    std::max(current.y, other.y);
                const int min_w = std::max(1, std::min(current.width, other.width));
                const int min_h = std::max(1, std::min(current.height, other.height));
                const int gap_x = std::max(
                    0,
                    std::max(current.x, other.x) -
                    std::min(current.x + current.width, other.x + other.width));
                const int gap_y = std::max(
                    0,
                    std::max(current.y, other.y) -
                    std::min(current.y + current.height, other.y + other.height));

                const bool strong_x_relation =
                    overlap_x >= static_cast<int>(0.35 * min_w) || gap_x <= merge_gap_x;
                const bool strong_y_relation =
                    overlap_y >= static_cast<int>(0.20 * min_h) || gap_y <= merge_gap_y;
                if (!strong_x_relation || !strong_y_relation) {
                    continue;
                }

                current |= other;
                area += merged[j].area;
                used[j] = 1;
                changed = true;
            }

            next_boxes.push_back(MotionBox{current, area});
        }
        merged = std::move(next_boxes);
    }

    return merged;
}

cv::Rect MotionDetector::padBox(const cv::Rect& box, int padding, const cv::Size& image_size)
{
    const int x0 = std::max(0, box.x - padding);
    const int y0 = std::max(0, box.y - padding);
    const int x1 = std::min(image_size.width, box.x + box.width + padding);
    const int y1 = std::min(image_size.height, box.y + box.height + padding);
    return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0));
}

double MotionDetector::rectIou(const cv::Rect2d& a, const cv::Rect2d& b)
{
    const double inter_w = std::max(0.0, std::min(a.x + a.width, b.x + b.width) -
                                          std::max(a.x, b.x));
    const double inter_h = std::max(0.0, std::min(a.y + a.height, b.y + b.height) -
                                          std::max(a.y, b.y));
    const double inter = inter_w * inter_h;
    const double area_a = std::max(0.0, a.width) * std::max(0.0, a.height);
    const double area_b = std::max(0.0, b.width) * std::max(0.0, b.height);
    const double union_area = area_a + area_b - inter;
    return union_area > 0.0 ? inter / union_area : 0.0;
}

double MotionDetector::boxCenterDistance(const cv::Rect2d& a, const cv::Rect2d& b)
{
    const double acx = a.x + a.width * 0.5;
    const double acy = a.y + a.height * 0.5;
    const double bcx = b.x + b.width * 0.5;
    const double bcy = b.y + b.height * 0.5;
    return std::hypot(acx - bcx, acy - bcy);
}

bool MotionDetector::boxesCanMatch(const cv::Rect2d& a, const cv::Rect2d& b)
{
    if (rectIou(a, b) >= 0.08) {
        return true;
    }
    const double max_dim = std::max(std::max(a.width, a.height), std::max(b.width, b.height));
    return boxCenterDistance(a, b) <= std::max(45.0, max_dim * 0.6);
}

cv::Rect2d MotionDetector::smoothBox(const cv::Rect2d& previous,
                                     const cv::Rect2d& detected,
                                     double alpha)
{
    alpha = std::max(0.0, std::min(1.0, alpha));
    const double shrink_alpha = alpha * 0.25;

    const double pcx = previous.x + previous.width * 0.5;
    const double pcy = previous.y + previous.height * 0.5;
    const double dcx = detected.x + detected.width * 0.5;
    const double dcy = detected.y + detected.height * 0.5;
    const double cx = pcx * (1.0 - alpha) + dcx * alpha;
    const double cy = pcy * (1.0 - alpha) + dcy * alpha;

    const double width_alpha = detected.width >= previous.width ? alpha : shrink_alpha;
    const double height_alpha = detected.height >= previous.height ? alpha : shrink_alpha;
    const double width = previous.width * (1.0 - width_alpha) + detected.width * width_alpha;
    const double height = previous.height * (1.0 - height_alpha) + detected.height * height_alpha;
    return cv::Rect2d(cx - width * 0.5, cy - height * 0.5, width, height);
}

cv::Rect2d MotionDetector::clipBox(const cv::Rect2d& box, const cv::Size& image_size)
{
    const double x0 = std::max(0.0, std::min(static_cast<double>(image_size.width), box.x));
    const double y0 = std::max(0.0, std::min(static_cast<double>(image_size.height), box.y));
    const double x1 = std::max(
        0.0,
        std::min(static_cast<double>(image_size.width), box.x + std::max(0.0, box.width)));
    const double y1 = std::max(
        0.0,
        std::min(static_cast<double>(image_size.height), box.y + std::max(0.0, box.height)));
    return cv::Rect2d(x0, y0, std::max(0.0, x1 - x0), std::max(0.0, y1 - y0));
}

std::vector<MotionDetector::MotionBox> MotionDetector::smoothTrackedBoxes(
    const std::vector<MotionBox>& boxes,
    const cv::Size& image_size)
{
    std::vector<cv::Rect2d> detections;
    detections.reserve(boxes.size());
    for (const auto& box : boxes) {
        const cv::Rect padded = padBox(box.box, config_.box_padding, image_size);
        if (padded.empty()) continue;
        detections.push_back(cv::Rect2d(padded.x, padded.y, padded.width, padded.height));
    }

    std::sort(detections.begin(), detections.end(), [](const cv::Rect2d& a, const cv::Rect2d& b) {
        return a.area() > b.area();
    });

    std::vector<uint8_t> used_tracks(tracked_boxes_.size(), 0);
    std::vector<TrackedMotionBox> next_tracks;
    next_tracks.reserve(detections.size() + tracked_boxes_.size());

    for (const auto& detection : detections) {
        int best_index = -1;
        double best_score = -1.0;
        for (size_t i = 0; i < tracked_boxes_.size(); ++i) {
            if (used_tracks[i] != 0) continue;
            const cv::Rect2d& track_box = tracked_boxes_[i].box;
            if (!boxesCanMatch(track_box, detection)) continue;

            const double iou = rectIou(track_box, detection);
            const double distance_score = 1.0 / (1.0 + boxCenterDistance(track_box, detection));
            const double score = iou + distance_score * 0.05;
            if (score > best_score) {
                best_score = score;
                best_index = static_cast<int>(i);
            }
        }

        cv::Rect2d output_box = detection;
        if (best_index >= 0) {
            used_tracks[best_index] = 1;
            output_box = smoothBox(tracked_boxes_[best_index].box, detection,
                                   config_.box_smoothing_alpha);
        }
        next_tracks.push_back(TrackedMotionBox{clipBox(output_box, image_size), 0});
    }

    for (size_t i = 0; i < tracked_boxes_.size(); ++i) {
        if (used_tracks[i] != 0) continue;
        TrackedMotionBox track = tracked_boxes_[i];
        track.missed_frames += 1;
        if (track.missed_frames <= config_.box_hold_frames) {
            track.box = clipBox(track.box, image_size);
            next_tracks.push_back(track);
        }
    }

    tracked_boxes_ = std::move(next_tracks);

    std::vector<MotionBox> smoothed_boxes;
    smoothed_boxes.reserve(tracked_boxes_.size());
    for (const auto& track : tracked_boxes_) {
        cv::Rect box(
            cv::saturate_cast<int>(std::round(track.box.x)),
            cv::saturate_cast<int>(std::round(track.box.y)),
            cv::saturate_cast<int>(std::round(track.box.width)),
            cv::saturate_cast<int>(std::round(track.box.height)));
        box &= cv::Rect(0, 0, image_size.width, image_size.height);
        if (box.empty()) continue;
        smoothed_boxes.push_back(MotionBox{box, static_cast<double>(box.area())});
    }

    return smoothed_boxes;
}

cv::Mat MotionDetector::buildDepthMask(const cv::Mat& depth, const cv::Size& output_size) const
{
    cv::Mat depth_mask(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
    const uint16_t min_mm = static_cast<uint16_t>(std::max(0.0, config_.depth_min_m) * 1000.0);
    const uint16_t max_mm = static_cast<uint16_t>(std::max(config_.depth_min_m, config_.depth_max_m) * 1000.0);

    for (int y = 0; y < depth.rows; ++y) {
        const uint16_t* depth_row = depth.ptr<uint16_t>(y);
        uint8_t* mask_row = depth_mask.ptr<uint8_t>(y);
        for (int x = 0; x < depth.cols; ++x) {
            const uint16_t d = depth_row[x];
            if (d > 0 && d >= min_mm && d <= max_mm && d <= 10000) {
                mask_row[x] = 255;
            }
        }
    }

    if (depth_mask.size() != output_size) {
        cv::Mat resized_mask;
        cv::resize(depth_mask, resized_mask, output_size, 0.0, 0.0, cv::INTER_NEAREST);
        return resized_mask;
    }
    return depth_mask;
}

void MotionDetector::process(const cv::Mat& src,
                             const cv::Mat& depth,
                             const std_msgs::Header& header)
{
    if (!config_.enabled || src.empty()) return;

    cv::Mat depth_mask(src.size(), CV_8UC1, cv::Scalar(255));
    const bool use_depth_mask =
        config_.depth_filter && !depth.empty() && config_.depth_max_m > 0.0;
    if (use_depth_mask) {
        depth_mask = buildDepthMask(depth, src.size());
        if (config_.depth_mask_dilate_iter > 0) {
            const cv::Mat depth_kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(config_.depth_mask_dilate_k, config_.depth_mask_dilate_k));
            cv::dilate(depth_mask, depth_mask, depth_kernel, cv::Point(-1, -1),
                       config_.depth_mask_dilate_iter);
        }
    }
    publishDebug(DebugImage::DepthMask, header, depth_mask);

    cv::Mat gaussian_original;
    cv::GaussianBlur(src, gaussian_original,
                     cv::Size(config_.gaussian_k, config_.gaussian_k),
                     config_.gaussian_sigma);

    cv::Mat gray_original;
    cv::cvtColor(gaussian_original, gray_original, cv::COLOR_BGR2GRAY);
    cv::Mat gray_enhanced = enhanceGray(gray_original, config_.gamma, config_.clahe_clip);
    cv::Mat stable_gray = gray_enhanced;
    if (config_.median_k > 1) {
        cv::medianBlur(gray_enhanced, stable_gray, config_.median_k);
    }

    cv::Mat masked_enhanced_gray;
    cv::bitwise_and(stable_gray, stable_gray, masked_enhanced_gray, depth_mask);
    publishDebug(DebugImage::Gray, header, masked_enhanced_gray);

    cv::Mat canny_mask = buildColorCanny(
        gaussian_original,
        stable_gray,
        config_.canny_low_threshold,
        config_.canny_high_threshold);

    cv::Mat masked_canny;
    cv::bitwise_and(canny_mask, canny_mask, masked_canny, depth_mask);
    publishDebug(DebugImage::Canny, header, masked_canny);

    const cv::Mat zeros = cv::Mat::zeros(masked_enhanced_gray.size(), CV_8UC1);
    cv::Mat frame_diff = zeros.clone();
    cv::Mat foreground_mask = zeros.clone();
    cv::Mat temporal_motion_mask = zeros.clone();
    cv::Mat motion_support_mask = zeros.clone();
    cv::Mat cleaned_motion_mask = zeros.clone();
    cv::Mat final_target_mask = zeros.clone();
    cv::Mat scene_foreground_mask = zeros.clone();
    cv::Mat scene_motion_edge_mask = zeros.clone();
    bool scene_motion_detected = false;

    const cv::Mat fg_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    if (prev_motion_gray_frame_.empty() ||
        prev_motion_gray_frame_.size() != masked_enhanced_gray.size()) {
        prev_motion_gray_frame_ = masked_enhanced_gray.clone();
        prev_scene_gray_frame_ = stable_gray.clone();
        prev_foreground_mask_ = zeros.clone();
        tracked_boxes_.clear();
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            last_boxes_.clear();
            last_motion_mask_.release();
            last_image_size_ = src.size();
        }
        publishDebug(DebugImage::FrameDiff, header, zeros);
        publishDebug(DebugImage::ForegroundRaw, header, zeros);
        publishDebug(DebugImage::ForegroundAfterDiff, header, zeros);
        publishDebug(DebugImage::MotionSupport, header, zeros);
        publishDebug(DebugImage::CleanedEdges, header, zeros);
        publishDebug(DebugImage::FinalMask, header, zeros);
        return;
    }

    cv::absdiff(prev_motion_gray_frame_, masked_enhanced_gray, frame_diff);
    publishDebug(DebugImage::FrameDiff, header, frame_diff);

    cv::threshold(frame_diff, foreground_mask, config_.diff_threshold, 255, cv::THRESH_BINARY);
    cv::bitwise_and(foreground_mask, foreground_mask, foreground_mask, depth_mask);
    cv::morphologyEx(foreground_mask, foreground_mask, cv::MORPH_OPEN, fg_kernel);
    cv::morphologyEx(foreground_mask, foreground_mask, cv::MORPH_CLOSE, fg_kernel);
    foreground_mask = removeSmallComponents(
        foreground_mask, static_cast<double>(config_.min_area));
    publishDebug(DebugImage::ForegroundRaw, header, foreground_mask);
    const double foreground_ratio =
        static_cast<double>(cv::countNonZero(foreground_mask)) /
        static_cast<double>(std::max(1, foreground_mask.rows * foreground_mask.cols));
    const bool foreground_too_large =
        config_.max_foreground_ratio > 0.0 &&
        foreground_ratio > config_.max_foreground_ratio;

    temporal_motion_mask = foreground_mask.clone();
    if (!prev_foreground_mask_.empty() &&
        prev_foreground_mask_.size() == foreground_mask.size()) {
        cv::Mat prev_support_mask;
        cv::dilate(prev_foreground_mask_, prev_support_mask, fg_kernel);
        cv::bitwise_and(foreground_mask, prev_support_mask, temporal_motion_mask);
    }
    temporal_motion_mask = removeSmallComponents(
        temporal_motion_mask, static_cast<double>(config_.min_area));
    publishDebug(DebugImage::ForegroundAfterDiff, header, temporal_motion_mask);

    if (!prev_scene_gray_frame_.empty() &&
        prev_scene_gray_frame_.size() == stable_gray.size()) {
        cv::Mat scene_diff_image;
        cv::absdiff(prev_scene_gray_frame_, stable_gray, scene_diff_image);
        cv::threshold(scene_diff_image, scene_foreground_mask,
                      config_.diff_threshold, 255, cv::THRESH_BINARY);
        cv::morphologyEx(scene_foreground_mask, scene_foreground_mask,
                         cv::MORPH_OPEN, fg_kernel);
        cv::morphologyEx(scene_foreground_mask, scene_foreground_mask,
                         cv::MORPH_CLOSE, fg_kernel);
        cv::bitwise_and(canny_mask, canny_mask, scene_motion_edge_mask,
                        scene_foreground_mask);

        const int total_scene_edge_pixels = std::max(1, cv::countNonZero(canny_mask));
        const double scene_change_ratio =
            static_cast<double>(cv::countNonZero(scene_motion_edge_mask)) /
            static_cast<double>(total_scene_edge_pixels);
        const double scene_spread_ratio = computeMotionSpread(scene_motion_edge_mask);
        scene_motion_detected =
            scene_change_ratio >= (config_.scene_motion_pct / 100.0) &&
            scene_spread_ratio >= 0.55;
    }

    if (!scene_motion_detected && !foreground_too_large) {
        motion_support_mask = temporal_motion_mask.clone();
        cleaned_motion_mask = temporal_motion_mask.clone();
        final_target_mask = closeAndFillMotionForeground(
            temporal_motion_mask,
            config_.merge_k,
            config_.merge_iter,
            static_cast<double>(config_.min_area));
    }

    publishDebug(DebugImage::MotionSupport, header, motion_support_mask);
    publishDebug(DebugImage::CleanedEdges, header, cleaned_motion_mask);
    publishDebug(DebugImage::FinalMask, header, final_target_mask);

    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_boxes_.clear();
        last_motion_mask_ = final_target_mask.clone();
        last_image_size_ = src.size();
    }

    prev_motion_gray_frame_ = masked_enhanced_gray.clone();
    prev_scene_gray_frame_ = stable_gray.clone();
    prev_foreground_mask_ = foreground_mask.clone();
}

void MotionDetector::drawCached(cv::Mat& dst) const
{
    if (!config_.enabled || dst.empty()) {
        return;
    }

    cv::Mat motion_mask;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (!last_motion_mask_.empty()) {
            motion_mask = last_motion_mask_.clone();
        }
    }
    if (motion_mask.empty() || cv::countNonZero(motion_mask) == 0) {
        return;
    }

    if (motion_mask.size() != dst.size()) {
        cv::resize(motion_mask, motion_mask, dst.size(), 0.0, 0.0, cv::INTER_NEAREST);
    }

    cv::Mat green_layer(dst.size(), dst.type(), cv::Scalar(0, 255, 0));
    cv::Mat blended;
    cv::addWeighted(dst, 0.55, green_layer, 0.45, 0.0, blended);
    blended.copyTo(dst, motion_mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(motion_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < config_.min_area) continue;

        const cv::Rect box = cv::boundingRect(contour) & cv::Rect(0, 0, dst.cols, dst.rows);
        if (box.empty()) continue;

        cv::rectangle(dst, box, cv::Scalar(0, 255, 0), config_.final_thick, cv::LINE_AA);

        const std::string label = "Moving";
        int baseline = 0;
        const cv::Size text_size =
            cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            config_.final_thick, &baseline);
        const int text_x = std::max(0, box.x);
        const int text_y = std::max(text_size.height + 8, box.y - 8);
        const cv::Point bg_tl(text_x, std::max(0, text_y - text_size.height - 6));
        const cv::Point bg_br(
            std::min(dst.cols - 1, text_x + text_size.width + 8),
            std::min(dst.rows - 1, text_y + baseline + 2));

        cv::rectangle(dst, bg_tl, bg_br, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(dst, label, cv::Point(text_x + 4, text_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0),
                    config_.final_thick, cv::LINE_AA);
    }
}

}  // namespace vision_pkg
