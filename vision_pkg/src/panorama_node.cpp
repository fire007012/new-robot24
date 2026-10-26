#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <mutex>
#include <string>
#include "vision_pkg/panorama.h"

class PanoramaNode
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    PanoramaNode() : nh_(std::make_shared<rclcpp::Node>("panorama_node"))
    {
        int output_queue_size;
        std::string cam1_topic;
        std::string cam2_topic;
        std::string display_mode;
        std::string input_transport;
        double fisheye_source_fov_deg;
        double operator_view_fov_deg;
        int operator_view_width;
        int operator_view_height;
        int front_crop_x;
        int front_crop_y;
        int front_crop_width;
        int front_crop_height;
        int front_crop_ref_width;
        int front_crop_ref_height;
        int back_crop_x;
        int back_crop_y;
        int back_crop_width;
        int back_crop_height;
        int back_crop_ref_width;
        int back_crop_ref_height;
        bool front_rotate_180;
        bool back_rotate_180;
        bool tcp_nodelay;

        cam1_topic = nh_->declare_parameter<std::string>("cam1_topic", "/forward_camera/image_raw");
        cam2_topic = nh_->declare_parameter<std::string>("cam2_topic", "/back_camera/image_raw");
        display_mode = nh_->declare_parameter<std::string>("display_mode", "azimuthal");
        input_transport = nh_->declare_parameter<std::string>("input_transport", "raw");
        fisheye_source_fov_deg = nh_->declare_parameter<double>("fisheye_source_fov_deg", 180.0);
        operator_view_fov_deg = nh_->declare_parameter<double>("operator_view_fov_deg", 120.0);
        operator_view_width = nh_->declare_parameter<int>("operator_view_width", 640);
        operator_view_height = nh_->declare_parameter<int>("operator_view_height", 360);
        front_rotate_180 = nh_->declare_parameter<bool>("front_rotate_180", false);
        back_rotate_180 = nh_->declare_parameter<bool>("back_rotate_180", true);
        front_crop_ref_width = nh_->declare_parameter<int>("front_crop_ref_width", 1920);
        front_crop_ref_height = nh_->declare_parameter<int>("front_crop_ref_height", 1080);
        front_crop_x = nh_->declare_parameter<int>("front_crop_x", 525);
        front_crop_y = nh_->declare_parameter<int>("front_crop_y", 276);
        front_crop_width = nh_->declare_parameter<int>("front_crop_width", 876);
        front_crop_height = nh_->declare_parameter<int>("front_crop_height", 538);
        back_crop_ref_width = nh_->declare_parameter<int>("back_crop_ref_width", 1920);
        back_crop_ref_height = nh_->declare_parameter<int>("back_crop_ref_height", 1080);
        back_crop_x = nh_->declare_parameter<int>("back_crop_x", 558);
        back_crop_y = nh_->declare_parameter<int>("back_crop_y", 250);
        back_crop_width = nh_->declare_parameter<int>("back_crop_width", 876);
        back_crop_height = nh_->declare_parameter<int>("back_crop_height", 538);
        tcp_nodelay = nh_->declare_parameter<bool>("tcp_nodelay", true);
        output_queue_size = nh_->declare_parameter<int>("output_queue_size", 1);

        pub_ = image_transport::create_publisher(nh_.get(), "panorama_image");
        equirect_pub_ = image_transport::create_publisher(nh_.get(), "panorama_equirect");

        panorama_.setFisheyeSourceFovDeg(static_cast<float>(fisheye_source_fov_deg));
        panorama_.setFrontRotate180(front_rotate_180);
        panorama_.setBackRotate180(back_rotate_180);
        panorama_.setFrontCropRect(front_crop_ref_width, front_crop_ref_height,
                                   front_crop_x, front_crop_y, front_crop_width, front_crop_height);
        panorama_.setBackCropRect(back_crop_ref_width, back_crop_ref_height,
                                  back_crop_x, back_crop_y, back_crop_width, back_crop_height);
        panorama_.setDisplayMode(display_mode);
        panorama_.setOperatorViewFovDeg(static_cast<float>(operator_view_fov_deg));
        panorama_.setOperatorViewSize(operator_view_width, operator_view_height);
        sub_cam1_ = image_transport::create_subscription(nh_.get(), cam1_topic, std::bind(&PanoramaNode::cam1Callback, this, std::placeholders::_1), input_transport, rmw_qos_profile_sensor_data);
        sub_cam2_ = image_transport::create_subscription(nh_.get(), cam2_topic, std::bind(&PanoramaNode::cam2Callback, this, std::placeholders::_1), input_transport, rmw_qos_profile_sensor_data);

        panorama_.setPanoramaReadyCallback(
            [this](const cv::Mat &primary_view, const cv::Mat &equirect) {
                rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    stamp = last_output_stamp_;
                }
                if ((stamp.nanoseconds()==0)) {
                    stamp = nh_->now();
                }
                if (!primary_view.empty()) {
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", primary_view).toImageMsg();
                    msg->header.stamp = stamp;
                    pub_.publish(msg);
                }
                if (!equirect.empty() && equirect_pub_.getNumSubscribers() > 0) {
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", equirect).toImageMsg();
                    msg->header.stamp = stamp;
                    equirect_pub_.publish(msg);
                }
            });

        RCLCPP_INFO_STREAM(rclcpp::get_logger("vision_pkg"), 
            "panorama_node started: cam1=" << cam1_topic
            << " cam2=" << cam2_topic
            << " mode=" << display_mode
            << " source_fov=" << fisheye_source_fov_deg
            << " view_fov=" << operator_view_fov_deg
            << " view_size=" << operator_view_width << "x" << operator_view_height
            << " input_transport=" << input_transport
            << " tcp_nodelay=" << (tcp_nodelay ? "true" : "false")
            << " output_queue=" << output_queue_size
            << " front_rotate_180=" << (front_rotate_180 ? "true" : "false")
            << " back_rotate_180=" << (back_rotate_180 ? "true" : "false")
            << " front_crop_ref=" << front_crop_ref_width << "x" << front_crop_ref_height
            << " front_crop=(" << front_crop_x << "," << front_crop_y << ","
            << front_crop_width << "," << front_crop_height << ")"
            << " back_crop_ref=" << back_crop_ref_width << "x" << back_crop_ref_height
            << " back_crop=(" << back_crop_x << "," << back_crop_y << ","
            << back_crop_width << "," << back_crop_height << ")");
    }

private:
    bool hasOutputSubscribers() const
    {
        return pub_.getNumSubscribers() > 0 || equirect_pub_.getNumSubscribers() > 0;
    }

    void cam1Callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            const auto image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_cam1_ = image;
            }
            if (!hasOutputSubscribers()) {
                return;
            }
            tryProcess();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "panorama cam1 decode error: %s", e.what());
        }
    }

    void cam2Callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            const auto image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_cam2_ = image;
            }
            if (!hasOutputSubscribers()) {
                return;
            }
            tryProcess();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "panorama cam2 decode error: %s", e.what());
        }
    }

    void tryProcess()
    {
        if (!hasOutputSubscribers()) {
            return;
        }

        cv_bridge::CvImageConstPtr cam1;
        cv_bridge::CvImageConstPtr cam2;
        rclcpp::Time output_stamp{0, 0, RCL_ROS_TIME};
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!latest_cam1_ || !latest_cam2_) {
                return;
            }

            const bool same_pair =
                latest_cam1_->header.stamp == last_processed_cam1_stamp_ &&
                latest_cam2_->header.stamp == last_processed_cam2_stamp_;
            if (same_pair) {
                return;
            }

            cam1 = latest_cam1_;
            cam2 = latest_cam2_;
            last_processed_cam1_stamp_ = cam1->header.stamp;
            last_processed_cam2_stamp_ = cam2->header.stamp;
            output_stamp =
                (rclcpp::Time(cam1->header.stamp) >= rclcpp::Time(cam2->header.stamp)) ? cam1->header.stamp : cam2->header.stamp;
            last_output_stamp_ = output_stamp;
        }

        panorama_.processFrames(cam1->image, cam2->image);
    }

    rclcpp::Node::SharedPtr nh_;
    
    image_transport::Publisher pub_;
    image_transport::Publisher equirect_pub_;
    image_transport::Subscriber sub_cam1_;
    image_transport::Subscriber sub_cam2_;
    Panorama panorama_;

    std::mutex frame_mutex_;
    cv_bridge::CvImageConstPtr latest_cam1_;
    cv_bridge::CvImageConstPtr latest_cam2_;
    rclcpp::Time last_processed_cam1_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_processed_cam2_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    PanoramaNode node;
    rclcpp::spin(node.node());
    return 0;
}
