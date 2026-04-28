#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>
#include "vision_pkg/Detection.h"
#include "vision_pkg/ObstacleWarning.h"
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
        nh_.param<double>("warning_distance", warning_distance_, 0.5);
        nh_.param<double>("use_rows_ratio", use_rows_ratio_, 0.6);
        nh_.param<bool>("use_cuda", use_cuda_, false);
        nh_.param<bool>("enable_qrcode_detection", enable_qrcode_detection_, false);
        nh_.param<bool>("enable_motion_detection", enable_motion_detection_, false);
        nh_.param<bool>("enable_motion_debug_images", enable_motion_debug_images_, false);
        nh_.param<bool>("enable_motion_depth_filter", enable_motion_depth_filter_, false);
        nh_.param<int>("motion_min_area", motion_min_area_, 80);
        nh_.param<double>("motion_canny_low_threshold", motion_canny_low_threshold_, 50.0);
        nh_.param<double>("motion_canny_high_threshold", motion_canny_high_threshold_, 150.0);
        nh_.param<double>("motion_learning_rate", motion_learning_rate_, 0.01);
        nh_.param<double>("motion_diff_threshold", motion_diff_threshold_, 18.0);
        nh_.param<double>("motion_max_foreground_ratio", motion_max_foreground_ratio_, 0.12);
        nh_.param<double>("motion_depth_min_m", motion_depth_min_m_, 0.0);
        nh_.param<double>("motion_depth_max_m", motion_depth_max_m_, 0.7);

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
            << " qrcode=" << (enable_qrcode_detection_ ? "true" : "false")
            << " motion=" << (enable_motion_detection_ ? "true" : "false")
            << " motion_debug=" << (enable_motion_debug_images_ ? "true" : "false")
            << " motion_depth_filter=" << (enable_motion_depth_filter_ ? "true" : "false"));
    }

private:
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

    void publishDebugMono(const image_transport::Publisher& pub,
                          const std_msgs::Header& header,
                          const cv::Mat& image) const
    {
        if (!enable_motion_debug_images_ || image.empty() || pub.getNumSubscribers() == 0) {
            return;
        }
        pub.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image).toImageMsg());
    }

    static std::string shortenText(const std::string& text, size_t max_len = 48)
    {
        if (text.size() <= max_len) return text;
        if (max_len <= 3) return text.substr(0, max_len);
        return text.substr(0, max_len - 3) + "...";
    }

    static bool extractQrCorners(const cv::Mat& points, int index, std::vector<cv::Point>& corners)
    {
        corners.clear();
        if (points.empty() || points.channels() != 2) return false;

        if (points.type() == CV_32FC2) {
            if (points.rows > index && points.cols >= 4) {
                for (int i = 0; i < 4; ++i) {
                    const cv::Vec2f p = points.at<cv::Vec2f>(index, i);
                    corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
                }
                return true;
            }
            if (index == 0 && points.rows >= 4 && points.cols == 1) {
                for (int i = 0; i < 4; ++i) {
                    const cv::Vec2f p = points.at<cv::Vec2f>(i, 0);
                    corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
                }
                return true;
            }
            if (index == 0 && points.rows == 1 && points.cols >= 4) {
                for (int i = 0; i < 4; ++i) {
                    const cv::Vec2f p = points.at<cv::Vec2f>(0, i);
                    corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
                }
                return true;
            }
        } else if (points.type() == CV_64FC2) {
            if (points.rows > index && points.cols >= 4) {
                for (int i = 0; i < 4; ++i) {
                    const cv::Vec2d p = points.at<cv::Vec2d>(index, i);
                    corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
                }
                return true;
            }
            if (index == 0 && points.rows >= 4 && points.cols == 1) {
                for (int i = 0; i < 4; ++i) {
                    const cv::Vec2d p = points.at<cv::Vec2d>(i, 0);
                    corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
                }
                return true;
            }
            if (index == 0 && points.rows == 1 && points.cols >= 4) {
                for (int i = 0; i < 4; ++i) {
                    const cv::Vec2d p = points.at<cv::Vec2d>(0, i);
                    corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
                }
                return true;
            }
        }

        return false;
    }

    void drawQrResult(cv::Mat& img, const std::vector<cv::Point>& corners, const std::string& decoded)
    {
        if (corners.size() != 4) return;

        std::vector<std::vector<cv::Point>> contour(1, corners);
        cv::polylines(img, contour, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        std::string text = decoded.empty() ? "QR" : "QR: " + shortenText(decoded);
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);

        int min_x = img.cols - 1;
        int min_y = img.rows - 1;
        for (const auto& p : corners) {
            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
        }

        int text_x = std::max(0, min_x);
        int text_y = std::max(text_size.height + 8, min_y - 8);
        int box_x2 = std::min(img.cols - 1, text_x + text_size.width + 8);
        int box_y1 = std::max(0, text_y - text_size.height - 6);
        int box_y2 = std::min(img.rows - 1, text_y + baseline + 2);

        cv::rectangle(img, cv::Point(text_x, box_y1), cv::Point(box_x2, box_y2),
                      cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(img, text, cv::Point(text_x + 4, text_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

    void annotateQRCodes(const cv::Mat& src, cv::Mat& dst)
    {
        if (!enable_qrcode_detection_ || src.empty()) return;

        std::vector<std::string> decoded_info;
        cv::Mat points;
        bool multi_ok = qr_detector_.detectAndDecodeMulti(src, decoded_info, points);

        if (multi_ok && !points.empty()) {
            int count = 0;
            if (points.rows > 0 && points.cols >= 4) {
                count = points.rows;
            } else if (points.rows >= 4 && points.cols == 1) {
                count = 1;
            }

            for (int i = 0; i < count; ++i) {
                std::vector<cv::Point> corners;
                if (!extractQrCorners(points, i, corners)) continue;
                const std::string decoded = (i < static_cast<int>(decoded_info.size())) ? decoded_info[i] : "";
                drawQrResult(dst, corners, decoded);
            }
            return;
        }

        cv::Mat single_points;
        const std::string decoded = qr_detector_.detectAndDecode(src, single_points);
        std::vector<cv::Point> corners;
        if (!decoded.empty() && extractQrCorners(single_points, 0, corners)) {
            drawQrResult(dst, corners, decoded);
        }
    }

    cv::Mat buildMotionDepthMask(const cv::Mat& depth, const cv::Size& output_size) const
    {
        cv::Mat depth_mask(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
        const uint16_t min_mm = static_cast<uint16_t>(std::max(0.0, motion_depth_min_m_) * 1000.0);
        const uint16_t max_mm = static_cast<uint16_t>(std::max(motion_depth_min_m_, motion_depth_max_m_) * 1000.0);

        for (int y = 0; y < depth.rows; ++y) {
            const uint16_t* depth_row = depth.ptr<uint16_t>(y);
            uint8_t* mask_row = depth_mask.ptr<uint8_t>(y);
            for (int x = 0; x < depth.cols; ++x) {
                const uint16_t d = depth_row[x];
                if (d >= min_mm && d <= max_mm && d <= 10000) {
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

    void annotateMotionObjects(const cv::Mat& src,
                               const cv::Mat& depth,
                               const std_msgs::Header& header,
                               cv::Mat& dst)
    {
        if (!enable_motion_detection_ || src.empty()) return;

        cv::Mat depth_mask(src.size(), CV_8UC1, cv::Scalar(255));
        const bool use_depth_mask =
            enable_motion_depth_filter_ && !depth.empty() && motion_depth_max_m_ > 0.0;
        if (use_depth_mask) {
            depth_mask = buildMotionDepthMask(depth, src.size());
        }
        publishDebugMono(motion_depth_mask_pub_, header, depth_mask);

        cv::Mat depth_filtered_src;
        if (use_depth_mask) {
            cv::bitwise_and(src, src, depth_filtered_src, depth_mask);
        } else {
            depth_filtered_src = src;
        }

        cv::Mat gray;
        cv::cvtColor(depth_filtered_src, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);
        publishDebugMono(motion_gray_pub_, header, gray);

        const double motion_alpha = std::max(0.0, std::min(1.0, motion_learning_rate_));
        cv::Mat canny_mask;
        cv::Canny(gray, canny_mask, motion_canny_low_threshold_, motion_canny_high_threshold_);

        if (motion_background_model_.empty() || motion_background_model_.size() != gray.size()) {
            gray.convertTo(motion_background_model_, CV_32FC1);
            const cv::Mat zeros = cv::Mat::zeros(gray.size(), CV_8UC1);
            publishDebugMono(motion_frame_diff_pub_, header, zeros);
            publishDebugMono(motion_fg_raw_pub_, header, zeros);
            publishDebugMono(motion_fg_after_diff_pub_, header, zeros);
            publishDebugMono(motion_fg_after_close_pub_, header, zeros);
            publishDebugMono(motion_fg_after_dilate_pub_, header, zeros);
            publishDebugMono(motion_fg_final_pub_, header, zeros);
            publishDebugMono(motion_fg_after_open_pub_, header, canny_mask);
            motion_frame_count_ = 1;
            return;
        }

        cv::Mat background_gray;
        motion_background_model_.convertTo(background_gray, CV_8UC1);
        publishDebugMono(motion_fg_after_open_pub_, header, canny_mask);

        cv::Mat frame_diff;
        cv::absdiff(gray, background_gray, frame_diff);
        publishDebugMono(motion_frame_diff_pub_, header, frame_diff);

        cv::Mat fg_mask;
        cv::threshold(frame_diff, fg_mask, motion_diff_threshold_, 255, cv::THRESH_BINARY);
        ++motion_frame_count_;
        publishDebugMono(motion_fg_raw_pub_, header, fg_mask);
        publishDebugMono(motion_fg_after_diff_pub_, header, fg_mask);

        cv::Mat motion_roi_mask;
        cv::Mat roi_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::dilate(fg_mask, motion_roi_mask, roi_kernel, cv::Point(-1, -1), 2);
        publishDebugMono(motion_fg_after_close_pub_, header, motion_roi_mask);

        cv::Mat moving_edge_mask;
        cv::bitwise_and(canny_mask, motion_roi_mask, moving_edge_mask);
        cv::Mat connected_edge_mask;
        cv::Mat connect_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(moving_edge_mask, connected_edge_mask, cv::MORPH_CLOSE, connect_kernel);
        cv::dilate(connected_edge_mask, connected_edge_mask, connect_kernel, cv::Point(-1, -1), 2);
        publishDebugMono(motion_fg_after_dilate_pub_, header, connected_edge_mask);
        publishDebugMono(motion_fg_final_pub_, header, connected_edge_mask);

        const double foreground_ratio =
            static_cast<double>(cv::countNonZero(fg_mask)) / static_cast<double>(fg_mask.total());
        if (foreground_ratio > motion_max_foreground_ratio_) {
            cv::accumulateWeighted(gray, motion_background_model_, std::max(0.05, motion_alpha));
            return;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(connected_edge_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const bool draw_output = !dst.empty();
        for (const auto& contour : contours) {
            const double area = cv::contourArea(contour);
            if (area < motion_min_area_) continue;
            if (!draw_output) continue;

            const cv::Rect box = cv::boundingRect(contour);
            cv::drawContours(dst, std::vector<std::vector<cv::Point>>(1, contour), -1,
                             cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::rectangle(dst, box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

            const std::string label = "Moving";
            int baseline = 0;
            const cv::Size text_size =
                cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
            const int text_x = std::max(0, box.x);
            const int text_y = std::max(text_size.height + 8, box.y - 8);
            const cv::Point bg_tl(text_x, std::max(0, text_y - text_size.height - 6));
            const cv::Point bg_br(
                std::min(dst.cols - 1, text_x + text_size.width + 8),
                std::min(dst.rows - 1, text_y + baseline + 2));

            cv::rectangle(dst, bg_tl, bg_br, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(dst, label, cv::Point(text_x + 4, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }

        cv::Mat background_update_mask;
        cv::bitwise_not(fg_mask, background_update_mask);
        cv::accumulateWeighted(gray, motion_background_model_, motion_alpha, background_update_mask);
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

    void colorCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
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
        const bool publish_image = image_pub_.getNumSubscribers() > 0;
        const bool need_yolo = has_yolo_ && (publish_image || det_pub_.getNumSubscribers() > 0);
        const bool need_motion = enable_motion_detection_ && (publish_image || hasMotionDebugSubscribers());
        const bool need_qrcode = enable_qrcode_detection_ && publish_image;
        const bool need_warning = has_depth && (publish_image || warning_pub_.getNumSubscribers() > 0);
        const bool need_center_distance = has_depth && publish_image;

        if (!publish_image && !need_yolo && !need_motion && !need_warning) {
            return;
        }

        if (publish_image && !need_yolo && !need_motion && !need_qrcode &&
            !need_warning && !need_center_distance) {
            image_pub_.publish(msg);
            return;
        }

        cv::Mat output;
        if (publish_image) {
            output = src.clone();
        }

        if (need_yolo) {
            std::vector<OutputParams> results;
            cv::Mat yolo_input = src;
            if (yolo_.detect(yolo_input, net_, results)) {
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
                }
                if (publish_image) {
                    output = yolo_.drawPred(output, results, yolo_.className, colors_);
                }

                if (publish_image && has_depth) {
                    for (const auto& r : results) {
                        float X;
                        float Y;
                        float Z;
                        if (!estimateObjectXYZ(depth, r.box, X, Y, Z)) continue;
                        char text[64];
                        snprintf(text, sizeof(text), "X:%.2f Y:%.2f Z:%.2fm", X, Y, Z);
                        int tx = std::max(0, r.box.x);
                        int ty = std::max(15, r.box.y - 6);
                        cv::putText(output, text, cv::Point(tx, ty),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                    cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
                        cv::putText(output, text, cv::Point(tx, ty),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                    }
                }
            }
        }

        if (need_motion) {
            annotateMotionObjects(src, depth, msg->header, output);
        }

        if (need_warning) {
            drawObstacleWarning(output, depth);
        }

        if (need_center_distance) {
            drawCenterDistance(output, depth);
        }

        if (need_qrcode) {
            annotateQRCodes(src, output);
        }

        if (publish_image) {
            image_pub_.publish(
                cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, output).toImageMsg());
        }
    }

    void drawObstacleWarning(cv::Mat& img, const cv::Mat& depth)
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

        if (img.empty()) {
            return;
        }

        double sx = static_cast<double>(img.cols) / cols;
        double sy = static_cast<double>(img.rows) / rows;
        int cy0 = static_cast<int>(y0 * sy);
        int cy1 = static_cast<int>(y1 * sy);
        int cthird = static_cast<int>(third * sx);

        cv::line(img, cv::Point(0, cy0), cv::Point(img.cols, cy0), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(0, cy1), cv::Point(img.cols, cy1), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cthird, cy0), cv::Point(cthird, cy1), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cthird * 2, cy0), cv::Point(cthird * 2, cy1), cv::Scalar(255, 255, 255), 1);

        double dists[3] = {left_dist, center_dist, right_dist};
        bool warns[3] = {warn.left_warn, warn.center_warn, warn.right_warn};
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

    void drawCenterDistance(cv::Mat& img, const cv::Mat& depth)
    {
        int cx = img.cols / 2;
        int cy = img.rows / 2;
        cv::line(img, cv::Point(cx - 20, cy), cv::Point(cx - 5, cy), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cx + 5, cy), cv::Point(cx + 20, cy), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cx, cy - 20), cv::Point(cx, cy - 5), cv::Scalar(255, 255, 255), 1);
        cv::line(img, cv::Point(cx, cy + 5), cv::Point(cx, cy + 20), cv::Scalar(255, 255, 255), 1);

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
        if (count > 0) {
            double dist = (sum / count) * 0.001;
            if (dist > 0.1) {
                char text[32];
                snprintf(text, sizeof(text), "%.2fm", dist);
                cv::putText(img, text, cv::Point(cx - 30, cy - 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
            }
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

    mutable std::mutex state_mutex_;
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
    Yolov8 yolo_;
    cv::dnn::Net net_;
    std::vector<cv::Scalar> colors_;
    cv::QRCodeDetector qr_detector_;
    bool enable_qrcode_detection_{false};
    cv::Mat motion_background_model_;
    bool enable_motion_detection_{false};
    bool enable_motion_debug_images_{false};
    bool enable_motion_depth_filter_{false};
    int motion_min_area_{80};
    int motion_frame_count_{0};
    double motion_canny_low_threshold_{50.0};
    double motion_canny_high_threshold_{150.0};
    double motion_learning_rate_{0.01};
    double motion_diff_threshold_{18.0};
    double motion_max_foreground_ratio_{0.12};
    double motion_depth_min_m_{0.0};
    double motion_depth_max_m_{0.7};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_display_node");
    VisionDisplayNode node;
    ros::spin();
    return 0;
}
