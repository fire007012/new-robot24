#include <vision_pkg/depth_image.hpp>
#include <rclcpp/rclcpp.hpp>
static rclcpp::Clock & vision_log_clock() { static rclcpp::Clock clock(RCL_STEADY_TIME); return clock; }
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <vector>
#include <vision_pkg/msg/obstacle_warning.hpp>

class ObstacleWarningNode
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    ObstacleWarningNode() : nh_(std::make_shared<rclcpp::Node>("obstacle_warning_node")), has_depth_(false)
    {
        std::string color_topic, depth_topic;
        color_topic = nh_->declare_parameter<std::string>("color_topic", "/behind_camera/color/image_raw");
        depth_topic = nh_->declare_parameter<std::string>("depth_topic", "/behind_camera/depth/image_rect_raw");
        warning_distance_ = nh_->declare_parameter<double>("warning_distance", 0.5);
        use_rows_ratio_ = nh_->declare_parameter<double>("use_rows_ratio", 0.6);

        warning_pub_ = nh_->create_publisher<vision_pkg::msg::ObstacleWarning>("obstacle_warning", 10);
        image_pub_ = image_transport::create_publisher(nh_.get(), "warning_image");

        color_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(color_topic, rclcpp::SensorDataQoS(), std::bind(&ObstacleWarningNode::colorCallback, this, std::placeholders::_1));
        depth_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(depth_topic, rclcpp::SensorDataQoS(), std::bind(&ObstacleWarningNode::depthCallback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "obstacle_warning_node started");
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  color_topic: %s", color_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  depth_topic: %s", depth_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  warning_distance: %.2f m", warning_distance_);
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  use_rows_ratio: %.2f", use_rows_ratio_);
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            depth_ptr_ = vision_pkg::depthMillimetres(msg);
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("vision_pkg"), vision_log_clock(), static_cast<int>((5.0)*1000), "depth cv_bridge error: %s", e.what());
        }
    }

    // 取区域第 5 百分位深度（排除噪点，取"最近"距离）
    double computeNearDist(const cv::Mat& depth, int x0, int y0, int x1, int y1)
    {
        std::vector<uint16_t> valid;
        valid.reserve((x1 - x0) * (y1 - y0));
        for (int y = y0; y < y1; ++y) {
            const uint16_t* row = depth.ptr<uint16_t>(y);
            for (int x = x0; x < x1; ++x) {
                uint16_t d = row[x];
                if (d > 0 && d <= 10000) valid.push_back(d);
            }
        }
        if (valid.empty()) return -1.0;

        // 第 5 百分位
        size_t idx = valid.size() * 5 / 100;
        if (idx >= valid.size()) idx = valid.size() - 1;
        std::nth_element(valid.begin(), valid.begin() + idx, valid.end());
        return valid[idx] * 0.001;
    }

    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (!has_depth_) return;

        const cv::Mat& depth = depth_ptr_->image;
        int cols = depth.cols;
        int rows = depth.rows;

        // 垂直方向只取中间 use_rows_ratio_ 的区域
        int margin = static_cast<int>(rows * (1.0 - use_rows_ratio_) / 2.0);
        int y0 = margin;
        int y1 = rows - margin;

        // 水平三等分
        int third = cols / 3;
        double left_dist   = computeNearDist(depth, 0,         y0, third,     y1);
        double center_dist = computeNearDist(depth, third,     y0, third * 2, y1);
        double right_dist  = computeNearDist(depth, third * 2, y0, cols,      y1);

        // 发布警告消息
        vision_pkg::msg::ObstacleWarning warn_msg;
        warn_msg.left_dist   = (left_dist   >= 0) ? left_dist   : 0;
        warn_msg.center_dist = (center_dist >= 0) ? center_dist : 0;
        warn_msg.right_dist  = (right_dist  >= 0) ? right_dist  : 0;
        warn_msg.left_warn   = (left_dist   >= 0 && left_dist   < warning_distance_);
        warn_msg.center_warn = (center_dist >= 0 && center_dist < warning_distance_);
        warn_msg.right_warn  = (right_dist  >= 0 && right_dist  < warning_distance_);
        warning_pub_->publish(warn_msg);

        // 可视化：在彩色图上画三个区域的状态
        if (image_pub_.getNumSubscribers() == 0) return;

        cv_bridge::CvImagePtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("vision_pkg"), vision_log_clock(), static_cast<int>((5.0)*1000), "color cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat& color = color_ptr->image;
        // 深度图和彩色图分辨率可能不同，计算缩放比例
        double sx = static_cast<double>(color.cols) / cols;
        double sy = static_cast<double>(color.rows) / rows;

        int cy0 = static_cast<int>(y0 * sy);
        int cy1 = static_cast<int>(y1 * sy);
        int cthird = static_cast<int>(third * sx);
        int ccols  = color.cols;

        struct Zone {
            const char* label;
            double dist;
            bool warn;
            int x0, x1;
        };
        Zone zones[3] = {
            {"L", left_dist,   warn_msg.left_warn,   0,          cthird},
            {"C", center_dist, warn_msg.center_warn,  cthird,     cthird * 2},
            {"R", right_dist,  warn_msg.right_warn,   cthird * 2, ccols}
        };

        // 细白线辅助框
        cv::Scalar line_color(255, 255, 255);
        int line_thickness = 1;

        // 画水平分割线
        cv::line(color, cv::Point(0, cy0), cv::Point(ccols, cy0), line_color, line_thickness);
        cv::line(color, cv::Point(0, cy1), cv::Point(ccols, cy1), line_color, line_thickness);

        // 画垂直分割线
        cv::line(color, cv::Point(cthird, cy0), cv::Point(cthird, cy1), line_color, line_thickness);
        cv::line(color, cv::Point(cthird * 2, cy0), cv::Point(cthird * 2, cy1), line_color, line_thickness);

        for (int i = 0; i < 3; ++i) {
            if (zones[i].dist < 0) continue;

            // 距离文字
            char text[64];
            snprintf(text, sizeof(text), "%.2fm", zones[i].dist);

            cv::Scalar text_color = zones[i].warn ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255);
            int font = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.6;
            int thickness = 1;
            int baseline = 0;
            cv::Size ts = cv::getTextSize(text, font, font_scale, thickness, &baseline);
            int tx = (zones[i].x0 + zones[i].x1 - ts.width) / 2;
            int ty = cy0 + 20;

            // 半透明黑色背景
            cv::Mat roi = color(cv::Rect(tx - 3, ty - ts.height - 2, ts.width + 6, ts.height + baseline + 4));
            cv::Mat overlay = roi.clone();
            cv::rectangle(overlay, cv::Point(0, 0), cv::Point(overlay.cols, overlay.rows), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::addWeighted(roi, 0.5, overlay, 0.5, 0, roi);

            cv::putText(color, text, cv::Point(tx, ty), font, font_scale, text_color, thickness);
        }

        image_pub_.publish(color_ptr->toImageMsg());
    }

    rclcpp::Node::SharedPtr nh_;
    
    image_transport::Publisher image_pub_;
    rclcpp::Publisher<vision_pkg::msg::ObstacleWarning>::SharedPtr warning_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    cv_bridge::CvImageConstPtr depth_ptr_;
    bool has_depth_;

    double warning_distance_;
    double use_rows_ratio_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ObstacleWarningNode node;
    rclcpp::spin(node.node());
    return 0;
}
