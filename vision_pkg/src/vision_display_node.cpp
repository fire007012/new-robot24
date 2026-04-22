#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <vector>
#include "vision_pkg/Detection.h"
#include "vision_pkg/ObstacleWarning.h"
#include "vision_pkg/yolov8.h"

class VisionDisplayNode
{
public:
    VisionDisplayNode() : nh_("~"), it_(nh_), has_depth_(false), has_intrinsics_(false)
    {
        std::string color_topic, depth_topic, camera_info_topic, model_path;
        nh_.param<std::string>("color_topic", color_topic, "/paw_camera/color/image_raw");
        nh_.param<std::string>("depth_topic", depth_topic, "/paw_camera/depth/image_rect_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic, "/paw_camera/color/camera_info");
        nh_.param<std::string>("model_path", model_path, "");
        nh_.param<double>("warning_distance", warning_distance_, 0.5);
        nh_.param<double>("use_rows_ratio", use_rows_ratio_, 0.6);
        nh_.param<bool>("use_cuda", use_cuda_, false);

        // Load YOLO
        has_yolo_ = false;
        if (!model_path.empty() && yolo_.ReadModel(net_, model_path, use_cuda_)) {
            has_yolo_ = true;
            for (size_t i = 0; i < yolo_.className.size(); i++) {
                colors_.push_back(cv::Scalar(rand() % 256, rand() % 256, rand() % 256));
            }
        }

        image_pub_ = it_.advertise("vision_image", 1);
        det_pub_ = nh_.advertise<vision_pkg::Detection>("detections", 10);
        warning_pub_ = nh_.advertise<vision_pkg::ObstacleWarning>("obstacle_warning", 10);

        color_sub_ = nh_.subscribe(color_topic, 1, &VisionDisplayNode::colorCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic, 1, &VisionDisplayNode::depthCallback, this);
        info_sub_ = nh_.subscribe(camera_info_topic, 1, &VisionDisplayNode::infoCallback, this);

        ROS_INFO("vision_display_node started");
    }

private:
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        try {
            depth_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(5.0, "depth error: %s", e.what());
        }
    }

    void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        fx_ = msg->K[0]; fy_ = msg->K[4];
        cx_ = msg->K[2]; cy_ = msg->K[5];
        color_w_ = msg->width;
        color_h_ = msg->height;
        has_intrinsics_ = (fx_ > 1e-3 && fy_ > 1e-3);
    }

    // 在 bbox 中心采样深度中位数，反投影为相机坐标系下的 XYZ（单位 m）。
    // 返回 false 表示样本不足或缺内参。
    bool estimateObjectXYZ(const cv::Mat& depth, const cv::Rect& box_color,
                           float& X, float& Y, float& Z)
    {
        if (!has_intrinsics_) return false;

        // color 与 depth 分辨率可能不同，按比例缩放 bbox 到 depth 坐标
        double sx = (double)depth.cols / std::max(1, color_w_);
        double sy = (double)depth.rows / std::max(1, color_h_);
        int bx = (int)(box_color.x * sx);
        int by = (int)(box_color.y * sy);
        int bw = std::max(1, (int)(box_color.width * sx));
        int bh = std::max(1, (int)(box_color.height * sy));

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

        // 用 color 图像素反投影（因为内参来自 color 相机）
        double u = box_color.x + box_color.width / 2.0;
        double v = box_color.y + box_color.height / 2.0;
        X = (float)((u - cx_) * z_m / fx_);
        Y = (float)((v - cy_) * z_m / fy_);
        Z = (float)z_m;
        return true;
    }

    double computeNearDist(const cv::Mat& depth, int x0, int y0, int x1, int y1)
    {
        std::vector<uint16_t> valid;
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
        if (!has_depth_) return;

        cv_bridge::CvImagePtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            return;
        }

        cv::Mat& img = color_ptr->image;
        const cv::Mat& depth = depth_ptr_->image;

        // 1. YOLO检测
        if (has_yolo_) {
            std::vector<OutputParams> results;
            if (yolo_.detect(img, net_, results)) {
                for (const auto& r : results) {
                    vision_pkg::Detection det;
                    det.id = r.id;
                    det.confidence = r.confidence;
                    det.x = r.box.x;
                    det.y = r.box.y;
                    det.width = r.box.width;
                    det.height = r.box.height;
                    if (r.id >= 0 && r.id < (int)yolo_.className.size())
                        det.class_name = yolo_.className[r.id];
                    det_pub_.publish(det);
                }
                img = yolo_.drawPred(img, results, yolo_.className, colors_);

                // 在每个检测框上方叠加 3D 坐标
                for (const auto& r : results) {
                    float X, Y, Z;
                    if (!estimateObjectXYZ(depth, r.box, X, Y, Z)) continue;
                    char text[64];
                    snprintf(text, sizeof(text), "X:%.2f Y:%.2f Z:%.2fm", X, Y, Z);
                    int tx = std::max(0, r.box.x);
                    int ty = std::max(15, r.box.y - 6);
                    cv::putText(img, text, cv::Point(tx, ty),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
                    cv::putText(img, text, cv::Point(tx, ty),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                }
            }
        }

        // 2. 障碍物警告区域
        drawObstacleWarning(img, depth);

        // 3. 中心距离显示
        drawCenterDistance(img, depth);

        image_pub_.publish(color_ptr->toImageMsg());
    }

    void drawObstacleWarning(cv::Mat& img, const cv::Mat& depth)
    {
        int cols = depth.cols, rows = depth.rows;
        int margin = static_cast<int>(rows * (1.0 - use_rows_ratio_) / 2.0);
        int y0 = margin, y1 = rows - margin, third = cols / 3;

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

        double sx = (double)img.cols / cols, sy = (double)img.rows / rows;
        int cy0 = y0 * sy, cy1 = y1 * sy, cthird = third * sx;

        cv::line(img, cv::Point(0, cy0), cv::Point(img.cols, cy0), cv::Scalar(255,255,255), 1);
        cv::line(img, cv::Point(0, cy1), cv::Point(img.cols, cy1), cv::Scalar(255,255,255), 1);
        cv::line(img, cv::Point(cthird, cy0), cv::Point(cthird, cy1), cv::Scalar(255,255,255), 1);
        cv::line(img, cv::Point(cthird*2, cy0), cv::Point(cthird*2, cy1), cv::Scalar(255,255,255), 1);

        double dists[3] = {left_dist, center_dist, right_dist};
        bool warns[3] = {warn.left_warn, warn.center_warn, warn.right_warn};
        int xs[3] = {cthird/2, cthird + cthird/2, cthird*2 + cthird/2};

        for (int i = 0; i < 3; ++i) {
            if (dists[i] < 0) continue;
            char text[32];
            snprintf(text, sizeof(text), "%.2fm", dists[i]);
            cv::Scalar color = warns[i] ? cv::Scalar(0,0,255) : cv::Scalar(255,255,255);
            cv::putText(img, text, cv::Point(xs[i]-20, cy0+20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 1);
        }
    }

    void drawCenterDistance(cv::Mat& img, const cv::Mat& depth)
    {
        int cx = img.cols / 2, cy = img.rows / 2;
        cv::line(img, cv::Point(cx-20, cy), cv::Point(cx-5, cy), cv::Scalar(255,255,255), 1);
        cv::line(img, cv::Point(cx+5, cy), cv::Point(cx+20, cy), cv::Scalar(255,255,255), 1);
        cv::line(img, cv::Point(cx, cy-20), cv::Point(cx, cy-5), cv::Scalar(255,255,255), 1);
        cv::line(img, cv::Point(cx, cy+5), cv::Point(cx, cy+20), cv::Scalar(255,255,255), 1);

        int dcx = depth.cols / 2, dcy = depth.rows / 2, half = 15;
        int count = 0;
        double sum = 0.0;
        for (int y = dcy - half; y <= dcy + half; ++y) {
            for (int x = dcx - half; x <= dcx + half; ++x) {
                if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows) continue;
                uint16_t d = depth.at<uint16_t>(y, x);
                if (d > 0 && d <= 10000) { sum += d; ++count; }
            }
        }
        if (count > 0) {
            double dist = (sum / count) * 0.001;
            if (dist > 0.1) {
                char text[32];
                snprintf(text, sizeof(text), "%.2fm", dist);
                cv::putText(img, text, cv::Point(cx-30, cy-30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 1);
            }
        }
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher det_pub_, warning_pub_;
    ros::Subscriber color_sub_, depth_sub_, info_sub_;
    cv_bridge::CvImageConstPtr depth_ptr_;
    bool has_depth_, has_yolo_, use_cuda_;
    bool has_intrinsics_;
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};
    int color_w_{0}, color_h_{0};
    double warning_distance_, use_rows_ratio_;
    Yolov8 yolo_;
    cv::dnn::Net net_;
    std::vector<cv::Scalar> colors_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_display_node");
    VisionDisplayNode node;
    ros::spin();
    return 0;
}


