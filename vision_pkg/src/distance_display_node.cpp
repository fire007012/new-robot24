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

class DistanceDisplayNode
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    DistanceDisplayNode() : nh_(std::make_shared<rclcpp::Node>("distance_display_node")), has_depth_(false)
    {
        std::string color_topic, depth_topic;
        color_topic = nh_->declare_parameter<std::string>("color_topic", "/paw_camera/color/image_raw");
        depth_topic = nh_->declare_parameter<std::string>("depth_topic", "/paw_camera/depth/image_rect_raw");
        distance_threshold_ = nh_->declare_parameter<double>("distance_threshold", 0.1);
        roi_size_ = nh_->declare_parameter<int>("roi_size", 30);

        image_pub_ = image_transport::create_publisher(nh_.get(), "distance_image");

        color_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(color_topic, rclcpp::SensorDataQoS(), std::bind(&DistanceDisplayNode::colorCallback, this, std::placeholders::_1));
        depth_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(depth_topic, rclcpp::SensorDataQoS(), std::bind(&DistanceDisplayNode::depthCallback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "distance_display_node started");
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  color_topic: %s", color_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  depth_topic: %s", depth_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  distance_threshold: %.2f m", distance_threshold_);
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "  roi_size: %d", roi_size_);
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

    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (!has_depth_) return;

        cv_bridge::CvImagePtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("vision_pkg"), vision_log_clock(), static_cast<int>((5.0)*1000), "color cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat& color = color_ptr->image;
        const cv::Mat& depth = depth_ptr_->image;

        int cx = depth.cols / 2;
        int cy = depth.rows / 2;
        int half = roi_size_ / 2;

        // 计算中心区域有效深度均值
        int count = 0;
        double sum = 0.0;
        for (int y = cy - half; y <= cy + half; ++y) {
            for (int x = cx - half; x <= cx + half; ++x) {
                if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows) continue;
                uint16_t d = depth.at<uint16_t>(y, x);
                if (d > 0 && d <= 10000) {
                    sum += d;
                    ++count;
                }
            }
        }

        // 画细白线十字准星
        int ccx = color.cols / 2;
        int ccy = color.rows / 2;
        int cross_len = 20;
        int gap = 5;
        cv::Scalar cross_color(255, 255, 255);
        int line_thickness = 1;

        // 上下左右四条线，中间留空隙
        cv::line(color, cv::Point(ccx - cross_len, ccy), cv::Point(ccx - gap, ccy), cross_color, line_thickness);
        cv::line(color, cv::Point(ccx + gap, ccy), cv::Point(ccx + cross_len, ccy), cross_color, line_thickness);
        cv::line(color, cv::Point(ccx, ccy - cross_len), cv::Point(ccx, ccy - gap), cross_color, line_thickness);
        cv::line(color, cv::Point(ccx, ccy + gap), cv::Point(ccx, ccy + cross_len), cross_color, line_thickness);

        // 距离超过阈值时显示
        if (count > 0) {
            double distance_m = (sum / count) * 0.001;
            if (distance_m > distance_threshold_) {
                char text[64];
                snprintf(text, sizeof(text), "%.2f m", distance_m);

                int font = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.7;
                int thickness = 1;
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);

                cv::Point text_org(ccx - text_size.width / 2, ccy - 30);

                // 半透明黑色背景
                cv::Mat roi = color(cv::Rect(text_org.x - 5, text_org.y - text_size.height - 3,
                                              text_size.width + 10, text_size.height + baseline + 6));
                cv::Mat overlay = roi.clone();
                cv::rectangle(overlay, cv::Point(0, 0), cv::Point(overlay.cols, overlay.rows),
                              cv::Scalar(0, 0, 0), cv::FILLED);
                cv::addWeighted(roi, 0.5, overlay, 0.5, 0, roi);

                cv::putText(color, text, text_org, font, font_scale, cv::Scalar(255, 255, 255), thickness);
            }
        }

        image_pub_.publish(color_ptr->toImageMsg());
    }

    rclcpp::Node::SharedPtr nh_;
    
    image_transport::Publisher image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    cv_bridge::CvImageConstPtr depth_ptr_;
    bool has_depth_;

    double distance_threshold_;
    int roi_size_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    DistanceDisplayNode node;
    rclcpp::spin(node.node());
    return 0;
}
