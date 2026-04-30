#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "vision_pkg/Detection.h"
#include "vision_pkg/ObstacleWarning.h"
#include "vision_pkg/SetDetectionEnabled.h"
#include "vision_pkg/motion_detector.h"
#include "vision_pkg/qr_detector.h"
#include "vision_pkg/yolov8.h"

class VisionDisplayNode
{
public:
    VisionDisplayNode() : nh_("~"), it_(nh_)
    {
        std::string color_topic;
        std::string depth_topic;
        std::string camera_info_topic;
        std::string model_path;
        std::string color_input_transport;
        std::string depth_input_transport;
        bool tcp_nodelay;
        int color_queue_size;
        int depth_queue_size;
        int info_queue_size;
        int output_queue_size;
        bool qrcode_require_decoded_text;

        nh_.param<std::string>("color_topic", color_topic, "/paw_camera/color/image_raw");
        nh_.param<std::string>("depth_topic", depth_topic, "/paw_camera/depth/image_rect_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic, "/paw_camera/color/camera_info");
        nh_.param<std::string>("model_path", model_path, "");
        nh_.param<std::string>("color_input_transport", color_input_transport, "raw");
        nh_.param<std::string>("depth_input_transport", depth_input_transport, "raw");
        nh_.param<bool>("tcp_nodelay", tcp_nodelay, true);
        nh_.param<int>("color_queue_size", color_queue_size, 1);
        nh_.param<int>("depth_queue_size", depth_queue_size, 1);
        nh_.param<int>("camera_info_queue_size", info_queue_size, 1);
        nh_.param<int>("output_queue_size", output_queue_size, 1);
        nh_.param<double>("output_max_fps", output_max_fps_, 30.0);
        nh_.param<double>("yolo_max_fps", yolo_max_fps_, 5.0);
        nh_.param<double>("motion_max_fps", motion_max_fps_, 15.0);
        nh_.param<double>("warning_max_fps", warning_max_fps_, 15.0);
        nh_.param<double>("center_distance_max_fps", center_distance_max_fps_, 15.0);
        nh_.param<double>("warning_distance", warning_distance_, 0.5);
        nh_.param<double>("use_rows_ratio", use_rows_ratio_, 0.6);
        nh_.param<bool>("use_cuda", use_cuda_, false);
        bool enable_yolo_detection = true;
        bool enable_qrcode_detection = false;
        bool enable_motion_detection = false;
        bool enable_obstacle_warning = true;
        bool enable_center_distance = true;
        nh_.param<bool>("enable_yolo_detection", enable_yolo_detection, true);
        nh_.param<bool>("enable_qrcode_detection", enable_qrcode_detection, false);
        nh_.param<bool>("enable_obstacle_warning", enable_obstacle_warning, true);
        nh_.param<bool>("enable_center_distance", enable_center_distance, true);
        nh_.param<double>("qrcode_eps_x", qrcode_eps_x_, 0.35);
        nh_.param<double>("qrcode_eps_y", qrcode_eps_y_, 0.35);
        nh_.param<bool>("qrcode_require_decoded_text", qrcode_require_decoded_text, true);
        nh_.param<bool>("enable_motion_detection", enable_motion_detection, false);
        nh_.param<bool>("enable_motion_debug_images", enable_motion_debug_images_, false);
        nh_.param<bool>("enable_motion_depth_filter", enable_motion_depth_filter_, false);
        nh_.param<int>("motion_min_area", motion_min_area_, 30);
        nh_.param<double>("motion_canny_low_threshold", motion_canny_low_threshold_, 50.0);
        nh_.param<double>("motion_canny_high_threshold", motion_canny_high_threshold_, 150.0);
        nh_.param<double>("motion_learning_rate", motion_learning_rate_, 0.01);
        nh_.param<double>("motion_diff_threshold", motion_diff_threshold_, 9.0);
        nh_.param<double>("motion_max_foreground_ratio", motion_max_foreground_ratio_, 0.12);
        nh_.param<double>("motion_depth_min_m", motion_depth_min_m_, 0.0);
        nh_.param<double>("motion_depth_max_m", motion_depth_max_m_, 0.7);
        nh_.param<int>("motion_gaussian_k", motion_gaussian_k_, 5);
        nh_.param<double>("motion_gaussian_sigma", motion_gaussian_sigma_, 1.0);
        nh_.param<double>("motion_gamma", motion_gamma_, 0.75);
        nh_.param<double>("motion_clahe_clip", motion_clahe_clip_, 2.0);
        nh_.param<int>("motion_depth_mask_dilate_k", motion_depth_mask_dilate_k_, 5);
        nh_.param<int>("motion_depth_mask_dilate_iter", motion_depth_mask_dilate_iter_, 1);
        nh_.param<int>("motion_roi_dilate_k", motion_roi_dilate_k_, 9);
        nh_.param<int>("motion_roi_dilate_iter", motion_roi_dilate_iter_, 6);
        nh_.param<int>("motion_merge_k", motion_merge_k_, 13);
        nh_.param<int>("motion_merge_iter", motion_merge_iter_, 3);
        nh_.param<double>("motion_scene_motion_pct", motion_scene_motion_pct_, 18.0);
        nh_.param<double>("motion_min_support_ratio", motion_min_support_ratio_, 0.03);
        nh_.param<int>("motion_min_support_pixels", motion_min_support_pixels_, 60);
        nh_.param<int>("motion_final_thick", motion_final_thick_, 2);
        nh_.param<double>("motion_min_edge_motion_ratio", motion_min_edge_motion_ratio_, 0.12);
        nh_.param<double>("motion_box_smoothing_alpha", motion_box_smoothing_alpha_, 0.35);
        nh_.param<int>("motion_box_hold_frames", motion_box_hold_frames_, 3);
        nh_.param<int>("motion_box_padding", motion_box_padding_, 6);

        motion_gaussian_k_ = ensureOdd(motion_gaussian_k_, 1);
        motion_depth_mask_dilate_k_ = ensureOdd(motion_depth_mask_dilate_k_, 1);
        motion_roi_dilate_k_ = ensureOdd(motion_roi_dilate_k_, 1);
        motion_merge_k_ = ensureOdd(motion_merge_k_, 3);
        motion_final_thick_ = std::max(1, motion_final_thick_);
        qrcode_eps_x_ = std::max(0.0, qrcode_eps_x_);
        qrcode_eps_y_ = std::max(0.0, qrcode_eps_y_);
        enable_yolo_detection_.store(enable_yolo_detection);
        enable_qrcode_detection_.store(enable_qrcode_detection);
        enable_motion_detection_.store(enable_motion_detection);
        enable_obstacle_warning_.store(enable_obstacle_warning);
        enable_center_distance_.store(enable_center_distance);
        qr_detector_.reset(new vision_pkg::QrDetector(
            true, qrcode_eps_x_, qrcode_eps_y_, qrcode_require_decoded_text));

        vision_pkg::MotionDetector::Config motion_config;
        motion_config.enabled = true;
        motion_config.debug_images = enable_motion_debug_images_;
        motion_config.depth_filter = enable_motion_depth_filter_;
        motion_config.min_area = motion_min_area_;
        motion_config.canny_low_threshold = motion_canny_low_threshold_;
        motion_config.canny_high_threshold = motion_canny_high_threshold_;
        motion_config.learning_rate = motion_learning_rate_;
        motion_config.diff_threshold = motion_diff_threshold_;
        motion_config.max_foreground_ratio = motion_max_foreground_ratio_;
        motion_config.depth_min_m = motion_depth_min_m_;
        motion_config.depth_max_m = motion_depth_max_m_;
        motion_config.gaussian_k = motion_gaussian_k_;
        motion_config.gaussian_sigma = motion_gaussian_sigma_;
        motion_config.gamma = motion_gamma_;
        motion_config.clahe_clip = motion_clahe_clip_;
        motion_config.depth_mask_dilate_k = motion_depth_mask_dilate_k_;
        motion_config.depth_mask_dilate_iter = motion_depth_mask_dilate_iter_;
        motion_config.roi_dilate_k = motion_roi_dilate_k_;
        motion_config.roi_dilate_iter = motion_roi_dilate_iter_;
        motion_config.merge_k = motion_merge_k_;
        motion_config.merge_iter = motion_merge_iter_;
        motion_config.scene_motion_pct = motion_scene_motion_pct_;
        motion_config.min_support_ratio = motion_min_support_ratio_;
        motion_config.min_support_pixels = motion_min_support_pixels_;
        motion_config.final_thick = motion_final_thick_;
        motion_config.min_edge_motion_ratio = motion_min_edge_motion_ratio_;
        motion_config.box_smoothing_alpha = motion_box_smoothing_alpha_;
        motion_config.box_hold_frames = motion_box_hold_frames_;
        motion_config.box_padding = motion_box_padding_;
        motion_detector_.reset(new vision_pkg::MotionDetector(
            motion_config,
            [this](vision_pkg::MotionDetector::DebugImage kind,
                   const std_msgs::Header& header,
                   const cv::Mat& image) {
                publishMotionDebugImage(kind, header, image);
            }));

        has_yolo_ = false;
        if (!model_path.empty() && yolo_.ReadModel(net_, model_path, use_cuda_)) {
            has_yolo_ = true;
            for (size_t i = 0; i < yolo_.className.size(); ++i) {
                colors_.push_back(cv::Scalar(rand() % 256, rand() % 256, rand() % 256));
            }
        }

        image_pub_ = it_.advertise("vision_image", output_queue_size);
        motion_gray_pub_ = it_.advertise("debug/motion_gray", 1);
        motion_frame_diff_pub_ = it_.advertise("debug/motion_frame_diff", 1);
        motion_fg_raw_pub_ = it_.advertise("debug/motion_fg_raw", 1);
        motion_fg_after_diff_pub_ = it_.advertise("debug/motion_fg_after_diff", 1);
        motion_fg_after_open_pub_ = it_.advertise("debug/motion_fg_after_open", 1);
        motion_fg_after_close_pub_ = it_.advertise("debug/motion_fg_after_close", 1);
        motion_fg_after_dilate_pub_ = it_.advertise("debug/motion_fg_after_dilate", 1);
        motion_depth_mask_pub_ = it_.advertise("debug/motion_depth_mask", 1);
        motion_fg_final_pub_ = it_.advertise("debug/motion_fg_final", 1);
        det_pub_ = nh_.advertise<vision_pkg::Detection>("detections", 10);
        warning_pub_ = nh_.advertise<vision_pkg::ObstacleWarning>("obstacle_warning", 10);
        detection_control_srv_ = nh_.advertiseService(
            "set_detection_enabled", &VisionDisplayNode::setDetectionEnabled, this);

        ros::TransportHints transport_hints;
        if (tcp_nodelay) {
            transport_hints = ros::TransportHints().tcpNoDelay();
        }

        image_transport::TransportHints color_hints(color_input_transport, transport_hints, nh_);
        image_transport::TransportHints depth_hints(depth_input_transport, transport_hints, nh_);
        color_sub_ = it_.subscribe(color_topic, color_queue_size,
                                   &VisionDisplayNode::colorCallback, this, color_hints);
        depth_sub_ = it_.subscribe(depth_topic, depth_queue_size,
                                   &VisionDisplayNode::depthCallback, this, depth_hints);
        info_sub_ = nh_.subscribe(camera_info_topic, info_queue_size,
                                  &VisionDisplayNode::infoCallback, this, transport_hints);

        ROS_INFO_STREAM(
            "vision_display_node started: color_topic=" << color_topic
            << " depth_topic=" << depth_topic
            << " camera_info_topic=" << camera_info_topic
            << " color_transport=" << color_input_transport
            << " depth_transport=" << depth_input_transport
            << " tcp_nodelay=" << (tcp_nodelay ? "true" : "false")
            << " queues=(" << color_queue_size << "," << depth_queue_size << ","
            << info_queue_size << "," << output_queue_size << ")"
            << " fps_limits=(output:" << output_max_fps_
            << " yolo:" << yolo_max_fps_
            << " motion:" << motion_max_fps_
            << " warning:" << warning_max_fps_
            << " center:" << center_distance_max_fps_ << ")"
            << " yolo=" << (enable_yolo_detection_.load() ? "true" : "false")
            << " qrcode=" << (enable_qrcode_detection_.load() ? "true" : "false")
            << " qrcode_mode=opencv410_detectAndDecodeMulti_current_frame"
            << " qrcode_eps=(" << qrcode_eps_x_ << "," << qrcode_eps_y_ << ")"
            << " qrcode_require_decoded_text="
            << (qrcode_require_decoded_text ? "true" : "false")
            << " motion=" << (enable_motion_detection_.load() ? "true" : "false")
            << " motion_debug=" << (enable_motion_debug_images_ ? "true" : "false")
            << " motion_depth_filter=" << (enable_motion_depth_filter_ ? "true" : "false")
            << " obstacle_warning=" << (enable_obstacle_warning_.load() ? "true" : "false")
            << " center_distance=" << (enable_center_distance_.load() ? "true" : "false"));

        if (has_yolo_) {
            yolo_worker_ = std::thread(&VisionDisplayNode::yoloWorkerLoop, this);
        }
        motion_worker_ = std::thread(&VisionDisplayNode::motionWorkerLoop, this);
    }

    ~VisionDisplayNode()
    {
        {
            std::lock_guard<std::mutex> lock(yolo_worker_mutex_);
            yolo_worker_stop_ = true;
        }
        yolo_worker_cv_.notify_one();
        {
            std::lock_guard<std::mutex> lock(motion_worker_mutex_);
            motion_worker_stop_ = true;
        }
        motion_worker_cv_.notify_one();
        if (yolo_worker_.joinable()) {
            yolo_worker_.join();
        }
        if (motion_worker_.joinable()) {
            motion_worker_.join();
        }
    }

private:
    struct CachedDetection
    {
        OutputParams result;
        bool has_xyz{false};
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    static bool isDue(const ros::Time& now, ros::Time& last_time, double max_fps)
    {
        if (max_fps <= 0.0) {
            return true;
        }
        if (last_time.isZero() || (now - last_time).toSec() >= (1.0 / max_fps)) {
            last_time = now;
            return true;
        }
        return false;
    }

    bool hasMotionDebugSubscribers() const
    {
        return motion_gray_pub_.getNumSubscribers() > 0 ||
               motion_frame_diff_pub_.getNumSubscribers() > 0 ||
               motion_fg_raw_pub_.getNumSubscribers() > 0 ||
               motion_fg_after_diff_pub_.getNumSubscribers() > 0 ||
               motion_fg_after_open_pub_.getNumSubscribers() > 0 ||
               motion_fg_after_close_pub_.getNumSubscribers() > 0 ||
               motion_fg_after_dilate_pub_.getNumSubscribers() > 0 ||
               motion_depth_mask_pub_.getNumSubscribers() > 0 ||
               motion_fg_final_pub_.getNumSubscribers() > 0;
    }

    bool setDetectionEnabled(vision_pkg::SetDetectionEnabled::Request& req,
                             vision_pkg::SetDetectionEnabled::Response& res)
    {
        const bool yolo_enabled = req.enable_all || req.enable_yolo;
        const bool qrcode_enabled = req.enable_all || req.enable_qrcode;
        const bool motion_enabled = req.enable_all || req.enable_motion;
        const bool warning_enabled = req.enable_all || req.enable_obstacle_warning;
        const bool center_enabled = req.enable_all || req.enable_center_distance;

        enable_yolo_detection_.store(yolo_enabled);
        enable_qrcode_detection_.store(qrcode_enabled);
        enable_motion_detection_.store(motion_enabled);
        enable_obstacle_warning_.store(warning_enabled);
        enable_center_distance_.store(center_enabled);

        nh_.setParam("enable_yolo_detection", yolo_enabled);
        nh_.setParam("enable_qrcode_detection", qrcode_enabled);
        nh_.setParam("enable_motion_detection", motion_enabled);
        nh_.setParam("enable_obstacle_warning", warning_enabled);
        nh_.setParam("enable_center_distance", center_enabled);

        if (!yolo_enabled) {
            clearYoloCache();
        }
        if (!motion_enabled && motion_detector_) {
            motion_detector_->clearCache();
        }
        if (!warning_enabled) {
            last_warning_valid_ = false;
        }
        if (!center_enabled) {
            last_center_distance_valid_ = false;
        }

        res.success = true;
        res.message = "detection switches updated";
        res.yolo_enabled = yolo_enabled;
        res.qrcode_enabled = qrcode_enabled;
        res.motion_enabled = motion_enabled;
        res.obstacle_warning_enabled = warning_enabled;
        res.center_distance_enabled = center_enabled;
        ROS_INFO_STREAM("detection switches updated: yolo=" << (yolo_enabled ? "true" : "false")
                        << " qrcode=" << (qrcode_enabled ? "true" : "false")
                        << " motion=" << (motion_enabled ? "true" : "false")
                        << " obstacle_warning=" << (warning_enabled ? "true" : "false")
                        << " center_distance=" << (center_enabled ? "true" : "false"));
        return true;
    }

    void publishDebugMono(const image_transport::Publisher& pub,
                          const std_msgs::Header& header,
                          const cv::Mat& image) const
    {
        if (!enable_motion_debug_images_ || image.empty() || pub.getNumSubscribers() == 0) {
            return;
        }
        pub.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image).toImageMsg());
    }

    void publishMotionDebugImage(vision_pkg::MotionDetector::DebugImage kind,
                                 const std_msgs::Header& header,
                                 const cv::Mat& image) const
    {
        switch (kind) {
        case vision_pkg::MotionDetector::DebugImage::Gray:
            publishDebugMono(motion_gray_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::FrameDiff:
            publishDebugMono(motion_frame_diff_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::ForegroundRaw:
            publishDebugMono(motion_fg_raw_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::ForegroundAfterDiff:
            publishDebugMono(motion_fg_after_diff_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::Canny:
            publishDebugMono(motion_fg_after_open_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::MotionSupport:
            publishDebugMono(motion_fg_after_close_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::CleanedEdges:
            publishDebugMono(motion_fg_after_dilate_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::DepthMask:
            publishDebugMono(motion_depth_mask_pub_, header, image);
            break;
        case vision_pkg::MotionDetector::DebugImage::FinalMask:
            publishDebugMono(motion_fg_final_pub_, header, image);
            break;
        }
    }

    std::vector<vision_pkg::QrDetector::Result> detectQRCodes(const cv::Mat& src)
    {
        if (!qr_detector_) {
            return {};
        }
        return qr_detector_->detect(src);
    }

    void drawQRCodes(cv::Mat& dst, const std::vector<vision_pkg::QrDetector::Result>& results)
    {
        if (qr_detector_) qr_detector_->draw(dst, results);
    }

    static int ensureOdd(int value, int minimum)
    {
        value = std::max(value, minimum);
        if (value % 2 == 0) {
            ++value;
        }
        return value;
    }

    void drawCachedMotionObjects(cv::Mat& dst) const
    {
        if (motion_detector_) motion_detector_->drawCached(dst);
    }

    bool hasCachedYoloDetections() const
    {
        std::lock_guard<std::mutex> lock(yolo_result_mutex_);
        return !last_detections_.empty();
    }

    void clearYoloCache()
    {
        std::lock_guard<std::mutex> lock(yolo_result_mutex_);
        last_detections_.clear();
        last_detection_image_size_ = cv::Size();
    }

    bool hasCachedMotionObjects() const
    {
        return motion_detector_ && motion_detector_->hasCachedResults();
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        try {
            const auto depth_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            std::lock_guard<std::mutex> lock(state_mutex_);
            depth_ptr_ = depth_ptr;
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_ONCE("depth error: %s", e.what());
        }
    }

    void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        color_w_ = msg->width;
        color_h_ = msg->height;
        has_intrinsics_ = (fx_ > 1e-3 && fy_ > 1e-3);
    }

    bool estimateObjectXYZ(const cv::Mat& depth, const cv::Rect& box_color,
                           float& X, float& Y, float& Z)
    {
        double fx;
        double fy;
        double cx;
        double cy;
        int color_w;
        int color_h;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!has_intrinsics_) return false;
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            color_w = color_w_;
            color_h = color_h_;
        }

        double sx = static_cast<double>(depth.cols) / std::max(1, color_w);
        double sy = static_cast<double>(depth.rows) / std::max(1, color_h);
        int bx = static_cast<int>(box_color.x * sx);
        int by = static_cast<int>(box_color.y * sy);
        int bw = std::max(1, static_cast<int>(box_color.width * sx));
        int bh = std::max(1, static_cast<int>(box_color.height * sy));

        int ucx = bx + bw / 2;
        int ucy = by + bh / 2;
        int half = std::min({bw / 4, bh / 4, 10});
        if (half < 2) half = 2;

        std::vector<uint16_t> vals;
        for (int y = std::max(0, ucy - half); y <= std::min(depth.rows - 1, ucy + half); ++y) {
            const uint16_t* row = depth.ptr<uint16_t>(y);
            for (int x = std::max(0, ucx - half); x <= std::min(depth.cols - 1, ucx + half); ++x) {
                uint16_t d = row[x];
                if (d > 0 && d <= 10000) vals.push_back(d);
            }
        }
        if (vals.size() < 5) return false;

        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
        double z_m = vals[vals.size() / 2] * 0.001;

        double u = box_color.x + box_color.width / 2.0;
        double v = box_color.y + box_color.height / 2.0;
        X = static_cast<float>((u - cx) * z_m / fx);
        Y = static_cast<float>((v - cy) * z_m / fy);
        Z = static_cast<float>(z_m);
        return true;
    }

    double computeNearDist(const cv::Mat& depth, int x0, int y0, int x1, int y1)
    {
        std::vector<uint16_t> valid;
        valid.reserve(std::max(0, x1 - x0) * std::max(0, y1 - y0));
        for (int y = y0; y < y1; ++y) {
            const uint16_t* row = depth.ptr<uint16_t>(y);
            for (int x = x0; x < x1; ++x) {
                uint16_t d = row[x];
                if (d > 0 && d <= 10000) valid.push_back(d);
            }
        }
        if (valid.empty()) return -1.0;
        size_t idx = valid.size() * 5 / 100;
        if (idx >= valid.size()) idx = valid.size() - 1;
        std::nth_element(valid.begin(), valid.begin() + idx, valid.end());
        return valid[idx] * 0.001;
    }

    void updateYoloDetections(const cv::Mat& src, const cv::Mat& depth)
    {
        if (!has_yolo_) {
            return;
        }

        std::vector<OutputParams> results;
        cv::Mat yolo_input = src;
        if (!yolo_.detect(yolo_input, net_, results)) {
            return;
        }

        std::vector<CachedDetection> detections;
        detections.reserve(results.size());

        for (const auto& r : results) {
            vision_pkg::Detection det;
            det.id = r.id;
            det.confidence = r.confidence;
            det.x = r.box.x;
            det.y = r.box.y;
            det.width = r.box.width;
            det.height = r.box.height;
            if (r.id >= 0 && r.id < static_cast<int>(yolo_.className.size())) {
                det.class_name = yolo_.className[r.id];
            }
            det_pub_.publish(det);

            CachedDetection cached;
            cached.result = r;
            if (!depth.empty()) {
                cached.has_xyz = estimateObjectXYZ(depth, r.box, cached.x, cached.y, cached.z);
            }
            detections.push_back(cached);
        }

        {
            std::lock_guard<std::mutex> lock(yolo_result_mutex_);
            last_detections_ = std::move(detections);
            last_detection_image_size_ = src.size();
        }
    }

    void drawCachedYoloDetections(cv::Mat& output)
    {
        if (!has_yolo_ || output.empty()) {
            return;
        }

        std::vector<CachedDetection> detections;
        cv::Size detection_image_size;
        {
            std::lock_guard<std::mutex> lock(yolo_result_mutex_);
            detections = last_detections_;
            detection_image_size = last_detection_image_size_;
        }
        if (detections.empty()) {
            return;
        }

        std::vector<OutputParams> results;
        std::vector<const CachedDetection*> visible_detections;
        results.reserve(detections.size());
        visible_detections.reserve(detections.size());
        const double sx = detection_image_size.width > 0
            ? static_cast<double>(output.cols) / detection_image_size.width
            : 1.0;
        const double sy = detection_image_size.height > 0
            ? static_cast<double>(output.rows) / detection_image_size.height
            : 1.0;

        for (const auto& cached : detections) {
            OutputParams r = cached.result;
            r.box.x = cv::saturate_cast<int>(r.box.x * sx);
            r.box.y = cv::saturate_cast<int>(r.box.y * sy);
            r.box.width = cv::saturate_cast<int>(r.box.width * sx);
            r.box.height = cv::saturate_cast<int>(r.box.height * sy);
            r.box &= cv::Rect(0, 0, output.cols, output.rows);
            if (!r.box.empty()) {
                results.push_back(r);
                visible_detections.push_back(&cached);
            }
        }

        if (!results.empty()) {
            output = yolo_.drawPred(output, results, yolo_.className, colors_);
        }

        for (size_t i = 0; i < visible_detections.size(); ++i) {
            const CachedDetection* cached = visible_detections[i];
            if (!cached || !cached->has_xyz) continue;
            const auto& r = results[i];
            char text[64];
            snprintf(text, sizeof(text), "X:%.2f Y:%.2f Z:%.2fm",
                     cached->x, cached->y, cached->z);
            int baseLine = 0;
            const double fontScale = 0.5;
            const int textThickness = 1;
            const int pad = 4;
            const cv::Size textSize = cv::getTextSize(
                text, cv::FONT_HERSHEY_SIMPLEX, fontScale, textThickness, &baseLine);

            const int maxTx = std::max(0, output.cols - textSize.width - pad * 2);
            const int tx = std::min(std::max(0, r.box.x), maxTx);
            int ty = r.box.y + r.box.height + textSize.height + pad + 2;
            if (ty + baseLine + pad >= output.rows) {
                ty = std::max(textSize.height + pad, r.box.y + r.box.height - pad);
            }

            const cv::Point bgTl(tx, std::max(0, ty - textSize.height - pad));
            const cv::Point bgBr(std::min(output.cols - 1, tx + textSize.width + pad * 2),
                                 std::min(output.rows - 1, ty + baseLine + pad));
            cv::rectangle(output, bgTl, bgBr, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(output, text, cv::Point(tx + pad, ty),
                        cv::FONT_HERSHEY_SIMPLEX, fontScale,
                        cv::Scalar(0, 255, 255), textThickness, cv::LINE_AA);
        }
    }

    void queueYoloDetection(const cv::Mat& src, const cv::Mat& depth)
    {
        if (!has_yolo_ || src.empty() || !yolo_worker_.joinable()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(yolo_worker_mutex_);
            yolo_pending_frame_ = src.clone();
            yolo_pending_depth_ = depth.empty() ? cv::Mat() : depth.clone();
            yolo_frame_pending_ = true;
        }
        yolo_worker_cv_.notify_one();
    }

    void yoloWorkerLoop()
    {
        while (ros::ok()) {
            cv::Mat frame;
            cv::Mat depth;
            {
                std::unique_lock<std::mutex> lock(yolo_worker_mutex_);
                yolo_worker_cv_.wait(lock, [this]() {
                    return yolo_worker_stop_ || yolo_frame_pending_;
                });
                if (yolo_worker_stop_) {
                    break;
                }
                frame = yolo_pending_frame_;
                depth = yolo_pending_depth_;
                yolo_pending_frame_.release();
                yolo_pending_depth_.release();
                yolo_frame_pending_ = false;
            }

            updateYoloDetections(frame, depth);
        }
    }

    void queueMotionDetection(const cv::Mat& src, const cv::Mat& depth, const std_msgs::Header& header)
    {
        if (!enable_motion_detection_.load() || src.empty() || !motion_worker_.joinable()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(motion_worker_mutex_);
            motion_pending_frame_ = src.clone();
            motion_pending_depth_ = depth.empty() ? cv::Mat() : depth.clone();
            motion_pending_header_ = header;
            motion_frame_pending_ = true;
        }
        motion_worker_cv_.notify_one();
    }

    void motionWorkerLoop()
    {
        while (ros::ok()) {
            cv::Mat frame;
            cv::Mat depth;
            std_msgs::Header header;
            {
                std::unique_lock<std::mutex> lock(motion_worker_mutex_);
                motion_worker_cv_.wait(lock, [this]() {
                    return motion_worker_stop_ || motion_frame_pending_;
                });
                if (motion_worker_stop_) {
                    break;
                }
                frame = motion_pending_frame_;
                depth = motion_pending_depth_;
                header = motion_pending_header_;
                motion_pending_frame_.release();
                motion_pending_depth_.release();
                motion_frame_pending_ = false;
            }

            if (motion_detector_) {
                motion_detector_->process(frame, depth, header);
            }
        }
    }

    void colorCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        const bool output_due = isDue(now, last_output_time_, output_max_fps_);

        cv_bridge::CvImageConstPtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_ONCE("color error: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr depth_ptr;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            depth_ptr = depth_ptr_;
        }

        const cv::Mat& src = color_ptr->image;
        const cv::Mat depth = depth_ptr ? depth_ptr->image : cv::Mat();
        const bool has_depth = !depth.empty();
        const bool yolo_enabled = enable_yolo_detection_.load();
        const bool motion_enabled = enable_motion_detection_.load();
        const bool qrcode_enabled = enable_qrcode_detection_.load();
        const bool warning_enabled = enable_obstacle_warning_.load();
        const bool center_enabled = enable_center_distance_.load();
        const bool publish_image = image_pub_.getNumSubscribers() > 0;
        const bool publish_this_frame = publish_image && output_due;
        const bool yolo_due = yolo_enabled && isDue(now, last_yolo_time_, yolo_max_fps_);
        const bool motion_due = motion_enabled && isDue(now, last_motion_time_, motion_max_fps_);
        const bool warning_due = warning_enabled && isDue(now, last_warning_time_, warning_max_fps_);
        const bool center_due = center_enabled && isDue(now, last_center_distance_time_, center_distance_max_fps_);
        const bool need_yolo = has_yolo_ && yolo_due && (publish_image || det_pub_.getNumSubscribers() > 0);
        const bool need_motion = motion_due &&
                                 (publish_image || hasMotionDebugSubscribers());
        const bool need_qrcode = qrcode_enabled && publish_this_frame;
        const bool need_warning = has_depth && warning_due &&
                                  (publish_image || warning_pub_.getNumSubscribers() > 0);
        const bool need_center_distance = has_depth && center_due && publish_image;

        if (!publish_this_frame && !need_yolo && !need_motion && !need_warning &&
            !need_center_distance && !need_qrcode) {
            return;
        }

        if (publish_this_frame && !need_yolo && !need_motion && !need_qrcode &&
            !need_warning && !need_center_distance) {
            if ((!yolo_enabled || !hasCachedYoloDetections()) &&
                (!motion_enabled || !hasCachedMotionObjects()) &&
                (!warning_enabled || !last_warning_valid_) &&
                (!center_enabled || !last_center_distance_valid_)) {
                image_pub_.publish(msg);
                return;
            }
        }

        if (need_motion) {
            queueMotionDetection(src, depth, msg->header);
        }

        if (need_warning) {
            updateObstacleWarning(depth);
        }

        if (need_center_distance) {
            updateCenterDistance(depth);
        }

        if (need_yolo) {
            queueYoloDetection(src, depth);
        }

        std::vector<vision_pkg::QrDetector::Result> qr_results;
        if (need_qrcode) {
            qr_results = detectQRCodes(src);
        }

        if (publish_this_frame) {
            cv::Mat output = src.clone();
            if (yolo_enabled) drawCachedYoloDetections(output);
            if (motion_enabled) drawCachedMotionObjects(output);
            if (warning_enabled) drawCachedObstacleWarning(output);
            if (center_enabled) drawCachedCenterDistance(output);
            if (need_qrcode) drawQRCodes(output, qr_results);
            image_pub_.publish(
                cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, output).toImageMsg());
        }
    }

    void updateObstacleWarning(const cv::Mat& depth)
    {
        int cols = depth.cols;
        int rows = depth.rows;
        int margin = static_cast<int>(rows * (1.0 - use_rows_ratio_) / 2.0);
        int y0 = margin;
        int y1 = rows - margin;
        int third = cols / 3;

        double left_dist = computeNearDist(depth, 0, y0, third, y1);
        double center_dist = computeNearDist(depth, third, y0, third * 2, y1);
        double right_dist = computeNearDist(depth, third * 2, y0, cols, y1);

        vision_pkg::ObstacleWarning warn;
        warn.left_dist = (left_dist >= 0) ? left_dist : 0;
        warn.center_dist = (center_dist >= 0) ? center_dist : 0;
        warn.right_dist = (right_dist >= 0) ? right_dist : 0;
        warn.left_warn = (left_dist >= 0 && left_dist < warning_distance_);
        warn.center_warn = (center_dist >= 0 && center_dist < warning_distance_);
        warn.right_warn = (right_dist >= 0 && right_dist < warning_distance_);
        warning_pub_.publish(warn);

        last_warning_valid_ = true;
        last_warning_depth_size_ = depth.size();
        last_left_dist_ = left_dist;
        last_center_dist_ = center_dist;
        last_right_dist_ = right_dist;
        last_left_warn_ = warn.left_warn != 0;
        last_center_warn_ = warn.center_warn != 0;
        last_right_warn_ = warn.right_warn != 0;
        last_warning_y0_ = y0;
        last_warning_y1_ = y1;
        last_warning_third_ = third;
    }

    void drawCachedObstacleWarning(cv::Mat& img) const
    {
        if (img.empty() || !last_warning_valid_) {
            return;
        }

        double sx = static_cast<double>(img.cols) / std::max(1, last_warning_depth_size_.width);
        double sy = static_cast<double>(img.rows) / std::max(1, last_warning_depth_size_.height);
        int cy0 = static_cast<int>(last_warning_y0_ * sy);
        int cy1 = static_cast<int>(last_warning_y1_ * sy);
        int cthird = static_cast<int>(last_warning_third_ * sx);

        cv::line(img, cv::Point(0, cy0), cv::Point(img.cols, cy0), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(0, cy1), cv::Point(img.cols, cy1), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cthird, cy0), cv::Point(cthird, cy1), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cthird * 2, cy0), cv::Point(cthird * 2, cy1), cv::Scalar(255, 255, 255), 1);
        double dists[3] = {last_left_dist_, last_center_dist_, last_right_dist_};
        bool warns[3] = {last_left_warn_, last_center_warn_, last_right_warn_};
        int xs[3] = {cthird / 2, cthird + cthird / 2, cthird * 2 + cthird / 2};

        for (int i = 0; i < 3; ++i) {
            if (dists[i] < 0) continue;
            char text[32];
            snprintf(text, sizeof(text), "%.2fm", dists[i]);
            cv::Scalar color = warns[i] ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255);
            cv::putText(img, text, cv::Point(xs[i] - 20, cy0 + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 1);
        }
    }

    void updateCenterDistance(const cv::Mat& depth)
    {
        int dcx = depth.cols / 2;
        int dcy = depth.rows / 2;
        int half = 15;
        int count = 0;
        double sum = 0.0;
        for (int y = dcy - half; y <= dcy + half; ++y) {
            for (int x = dcx - half; x <= dcx + half; ++x) {
                if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows) continue;
                uint16_t d = depth.at<uint16_t>(y, x);
                if (d > 0 && d <= 10000) {
                    sum += d;
                    ++count;
                }
            }
        }
        last_center_distance_valid_ = false;
        if (count > 0) {
            double dist = (sum / count) * 0.001;
            if (dist > 0.1) {
                last_center_distance_ = dist;
                last_center_distance_valid_ = true;
            }
        }
    }

    void drawCachedCenterDistance(cv::Mat& img) const
    {
        if (img.empty()) {
            return;
        }

        int cx = img.cols / 2;
        int cy = img.rows / 2;
        cv::line(img, cv::Point(cx - 20, cy), cv::Point(cx - 5, cy), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cx + 5, cy), cv::Point(cx + 20, cy), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cx, cy - 20), cv::Point(cx, cy - 5), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cx, cy + 5), cv::Point(cx, cy + 20), cv::Scalar(255, 255, 255), 1);

        if (last_center_distance_valid_) {
            char text[32];
            snprintf(text, sizeof(text), "%.2fm", last_center_distance_);
            cv::putText(img, text, cv::Point(cx - 30, cy - 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
        }
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher motion_gray_pub_;
    image_transport::Publisher motion_frame_diff_pub_;
    image_transport::Publisher motion_fg_raw_pub_;
    image_transport::Publisher motion_fg_after_diff_pub_;
    image_transport::Publisher motion_fg_after_open_pub_;
    image_transport::Publisher motion_fg_after_close_pub_;
    image_transport::Publisher motion_fg_after_dilate_pub_;
    image_transport::Publisher motion_depth_mask_pub_;
    image_transport::Publisher motion_fg_final_pub_;
    ros::Publisher det_pub_;
    ros::Publisher warning_pub_;
    image_transport::Subscriber color_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber info_sub_;
    ros::ServiceServer detection_control_srv_;

    mutable std::mutex state_mutex_;
    mutable std::mutex yolo_result_mutex_;
    std::thread yolo_worker_;
    std::mutex yolo_worker_mutex_;
    std::condition_variable yolo_worker_cv_;
    bool yolo_worker_stop_{false};
    bool yolo_frame_pending_{false};
    cv::Mat yolo_pending_frame_;
    cv::Mat yolo_pending_depth_;
    std::thread motion_worker_;
    std::mutex motion_worker_mutex_;
    std::condition_variable motion_worker_cv_;
    bool motion_worker_stop_{false};
    bool motion_frame_pending_{false};
    cv::Mat motion_pending_frame_;
    cv::Mat motion_pending_depth_;
    std_msgs::Header motion_pending_header_;
    cv_bridge::CvImageConstPtr depth_ptr_;
    bool has_yolo_{false};
    bool use_cuda_{false};
    bool has_intrinsics_{false};
    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    int color_w_{0};
    int color_h_{0};
    double warning_distance_{0.5};
    double use_rows_ratio_{0.6};
    double output_max_fps_{30.0};
    double yolo_max_fps_{10.0};
    double motion_max_fps_{20.0};
    double warning_max_fps_{15.0};
    double center_distance_max_fps_{15.0};
    double qrcode_eps_x_{0.35};
    double qrcode_eps_y_{0.35};
    ros::Time last_output_time_;
    ros::Time last_yolo_time_;
    ros::Time last_motion_time_;
    ros::Time last_warning_time_;
    ros::Time last_center_distance_time_;
    Yolov8 yolo_;
    cv::dnn::Net net_;
    std::vector<cv::Scalar> colors_;
    std::vector<CachedDetection> last_detections_;
    cv::Size last_detection_image_size_;
    std::atomic<bool> enable_yolo_detection_{true};
    std::atomic<bool> enable_qrcode_detection_{false};
    std::unique_ptr<vision_pkg::QrDetector> qr_detector_;
    std::unique_ptr<vision_pkg::MotionDetector> motion_detector_;
    std::atomic<bool> enable_motion_detection_{false};
    bool enable_motion_debug_images_{false};
    bool enable_motion_depth_filter_{false};
    std::atomic<bool> enable_obstacle_warning_{true};
    std::atomic<bool> enable_center_distance_{true};
    int motion_min_area_{30};
    double motion_canny_low_threshold_{50.0};
    double motion_canny_high_threshold_{150.0};
    double motion_learning_rate_{0.01};
    double motion_diff_threshold_{9.0};
    double motion_max_foreground_ratio_{0.12};
    double motion_depth_min_m_{0.0};
    double motion_depth_max_m_{0.7};
    int motion_gaussian_k_{5};
    double motion_gaussian_sigma_{1.0};
    double motion_gamma_{0.75};
    double motion_clahe_clip_{2.0};
    int motion_depth_mask_dilate_k_{5};
    int motion_depth_mask_dilate_iter_{1};
    int motion_roi_dilate_k_{9};
    int motion_roi_dilate_iter_{6};
    int motion_merge_k_{13};
    int motion_merge_iter_{3};
    double motion_scene_motion_pct_{18.0};
    double motion_min_support_ratio_{0.03};
    int motion_min_support_pixels_{60};
    int motion_final_thick_{2};
    double motion_min_edge_motion_ratio_{0.12};
    double motion_box_smoothing_alpha_{0.35};
    int motion_box_hold_frames_{3};
    int motion_box_padding_{6};
    bool last_warning_valid_{false};
    cv::Size last_warning_depth_size_;
    double last_left_dist_{-1.0};
    double last_center_dist_{-1.0};
    double last_right_dist_{-1.0};
    bool last_left_warn_{false};
    bool last_center_warn_{false};
    bool last_right_warn_{false};
    int last_warning_y0_{0};
    int last_warning_y1_{0};
    int last_warning_third_{0};
    bool last_center_distance_valid_{false};
    double last_center_distance_{0.0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_display_node");
    VisionDisplayNode node;
    ros::spin();
    return 0;
}
