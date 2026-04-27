#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <mutex>
#include <string>
#include "vision_pkg/panorama.h"

class PanoramaNode
{
public:
    PanoramaNode() : nh_("~"), it_(nh_)
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

        nh_.param<std::string>("cam1_topic", cam1_topic, "/forward_camera/image_raw");
        nh_.param<std::string>("cam2_topic", cam2_topic, "/back_camera/image_raw");
        nh_.param<std::string>("display_mode", display_mode, "azimuthal");
        nh_.param<std::string>("input_transport", input_transport, "raw");
        nh_.param<double>("fisheye_source_fov_deg", fisheye_source_fov_deg, 180.0);
        nh_.param<double>("operator_view_fov_deg", operator_view_fov_deg, 120.0);
        nh_.param<int>("operator_view_width", operator_view_width, 640);
        nh_.param<int>("operator_view_height", operator_view_height, 360);
        nh_.param<bool>("front_rotate_180", front_rotate_180, false);
        nh_.param<bool>("back_rotate_180", back_rotate_180, true);
        nh_.param<int>("front_crop_ref_width", front_crop_ref_width, 1920);
        nh_.param<int>("front_crop_ref_height", front_crop_ref_height, 1080);
        nh_.param<int>("front_crop_x", front_crop_x, 525);
        nh_.param<int>("front_crop_y", front_crop_y, 276);
        nh_.param<int>("front_crop_width", front_crop_width, 876);
        nh_.param<int>("front_crop_height", front_crop_height, 538);
        nh_.param<int>("back_crop_ref_width", back_crop_ref_width, 1920);
        nh_.param<int>("back_crop_ref_height", back_crop_ref_height, 1080);
        nh_.param<int>("back_crop_x", back_crop_x, 558);
        nh_.param<int>("back_crop_y", back_crop_y, 250);
        nh_.param<int>("back_crop_width", back_crop_width, 876);
        nh_.param<int>("back_crop_height", back_crop_height, 538);
        nh_.param<bool>("tcp_nodelay", tcp_nodelay, true);
        nh_.param<int>("output_queue_size", output_queue_size, 1);

        pub_ = it_.advertise("panorama_image", output_queue_size);
        equirect_pub_ = it_.advertise("panorama_equirect", output_queue_size);

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

        ros::TransportHints transport_hints;
        if (tcp_nodelay) {
            transport_hints = ros::TransportHints().tcpNoDelay();
        }
        image_transport::TransportHints image_hints(input_transport, transport_hints, nh_);
        sub_cam1_ = it_.subscribe(cam1_topic, 1, &PanoramaNode::cam1Callback, this, image_hints);
        sub_cam2_ = it_.subscribe(cam2_topic, 1, &PanoramaNode::cam2Callback, this, image_hints);

        panorama_.setPanoramaReadyCallback(
            [this](const cv::Mat &primary_view, const cv::Mat &equirect) {
                ros::Time stamp;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    stamp = last_output_stamp_;
                }
                if (stamp.isZero()) {
                    stamp = ros::Time::now();
                }
                if (!primary_view.empty()) {
                    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", primary_view).toImageMsg();
                    msg->header.stamp = stamp;
                    pub_.publish(msg);
                }
                if (!equirect.empty() && equirect_pub_.getNumSubscribers() > 0) {
                    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", equirect).toImageMsg();
                    msg->header.stamp = stamp;
                    equirect_pub_.publish(msg);
                }
            });

        ROS_INFO_STREAM(
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

    void cam1Callback(const sensor_msgs::ImageConstPtr& msg)
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
            ROS_WARN_ONCE("panorama cam1 decode error: %s", e.what());
        }
    }

    void cam2Callback(const sensor_msgs::ImageConstPtr& msg)
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
            ROS_WARN_ONCE("panorama cam2 decode error: %s", e.what());
        }
    }

    void tryProcess()
    {
        if (!hasOutputSubscribers()) {
            return;
        }

        cv_bridge::CvImageConstPtr cam1;
        cv_bridge::CvImageConstPtr cam2;
        ros::Time output_stamp;
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
                (cam1->header.stamp >= cam2->header.stamp) ? cam1->header.stamp : cam2->header.stamp;
            last_output_stamp_ = output_stamp;
        }

        panorama_.processFrames(cam1->image, cam2->image);
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    image_transport::Publisher equirect_pub_;
    image_transport::Subscriber sub_cam1_;
    image_transport::Subscriber sub_cam2_;
    Panorama panorama_;

    std::mutex frame_mutex_;
    cv_bridge::CvImageConstPtr latest_cam1_;
    cv_bridge::CvImageConstPtr latest_cam2_;
    ros::Time last_processed_cam1_stamp_;
    ros::Time last_processed_cam2_stamp_;
    ros::Time last_output_stamp_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panorama_node");
    PanoramaNode node;
    ros::spin();
    return 0;
}
