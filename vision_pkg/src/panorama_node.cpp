#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "vision_pkg/panorama.h"

class PanoramaNode
{
public:
    PanoramaNode() : nh_("~"), it_(nh_)
    {
        pub_ = it_.advertise("panorama_image", 1);
        equirect_pub_ = it_.advertise("panorama_equirect", 1);

        std::string cam1_topic, cam2_topic;
        nh_.param<std::string>("cam1_topic", cam1_topic, "/forward_camera/image_raw");
        nh_.param<std::string>("cam2_topic", cam2_topic, "/back_camera/image_raw");
        double fisheye_source_fov_deg;
        nh_.param<double>("fisheye_source_fov_deg", fisheye_source_fov_deg, 180.0);
        panorama_.setFisheyeSourceFovDeg(static_cast<float>(fisheye_source_fov_deg));

        sub_cam1_ = it_.subscribe(cam1_topic, 1, &PanoramaNode::cam1Callback, this);
        sub_cam2_ = it_.subscribe(cam2_topic, 1, &PanoramaNode::cam2Callback, this);

        panorama_.setPanoramaReadyCallback(
            [this](const cv::Mat &azimuthal, const cv::Mat &equirect) {
                if (!azimuthal.empty()) {
                    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", azimuthal).toImageMsg();
                    msg->header.stamp = ros::Time::now();
                    pub_.publish(msg);
                }
                if (!equirect.empty() && equirect_pub_.getNumSubscribers() > 0) {
                    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", equirect).toImageMsg();
                    msg->header.stamp = ros::Time::now();
                    equirect_pub_.publish(msg);
                }
            });

        ROS_INFO("panorama_node started");
        ROS_INFO("  cam1: %s", cam1_topic.c_str());
        ROS_INFO("  cam2: %s", cam2_topic.c_str());
        ROS_INFO("  fisheye_source_fov_deg: %.1f", fisheye_source_fov_deg);
        ROS_INFO("  azimuthal:  ~panorama_image");
        ROS_INFO("  equirect:   ~panorama_equirect");
    }

private:
    void cam1Callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        panorama_.receiveCam1(frame);
    }

    void cam2Callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        panorama_.receiveCam2(frame);
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    image_transport::Publisher equirect_pub_;
    image_transport::Subscriber sub_cam1_;
    image_transport::Subscriber sub_cam2_;
    Panorama panorama_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panorama_node");
    PanoramaNode node;
    ros::spin();
    return 0;
}

