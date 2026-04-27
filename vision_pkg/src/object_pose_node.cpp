#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <vector>
#include "vision_pkg/Detection.h"
#include "vision_pkg/DetectedObject3D.h"
#include "vision_pkg/DetectedObject3DArray.h"

class ObjectPoseNode
{
public:
    ObjectPoseNode() : nh_("~"), has_camera_info_(false), has_depth_(false), has_pose_(false), first_pose_(true)
    {
        std::string detection_topic, depth_topic, camera_info_topic;
        nh_.param<std::string>("detection_topic", detection_topic, "/yolov8/detections");
        nh_.param<std::string>("depth_topic", depth_topic, "/paw_camera/depth/image_rect_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic, "/paw_camera/color/camera_info");
        nh_.param<int>("publish_rate", publish_rate_, 20);
        nh_.param<double>("smooth_alpha", smooth_alpha_, 0.3);

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detected_object_pose", 10);
        array_pub_ = nh_.advertise<vision_pkg::DetectedObject3DArray>("detected_objects_3d", 10);

        det_sub_ = nh_.subscribe(detection_topic, 10, &ObjectPoseNode::detectionCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic, 1, &ObjectPoseNode::depthCallback, this);
        info_sub_ = nh_.subscribe(camera_info_topic, 1, &ObjectPoseNode::cameraInfoCallback, this);

        ROS_INFO_STREAM(
            "object_pose_node started: detection_topic=" << detection_topic
            << " depth_topic=" << depth_topic
            << " camera_info_topic=" << camera_info_topic
            << " publish_rate=" << publish_rate_
            << " smooth_alpha=" << smooth_alpha_);
    }

    void run()
    {
        ros::Rate rate(publish_rate_);
        while (ros::ok()) {
            ros::spinOnce();
            if (has_pose_) {
                cached_pose_.header.stamp = ros::Time::now();
                pose_pub_.publish(cached_pose_);
            }
            if (!pending_objects_.empty()) {
                vision_pkg::DetectedObject3DArray arr;
                arr.header.stamp = ros::Time::now();
                arr.header.frame_id = "paw_camera_color_optical_frame";
                arr.objects = std::move(pending_objects_);
                array_pub_.publish(arr);
                pending_objects_.clear();
            }
            rate.sleep();
        }
    }

private:
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        if (has_camera_info_) return;

        // K matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        has_camera_info_ = true;

        info_sub_.shutdown();
        ROS_INFO("Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        try {
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            ROS_WARN_ONCE("depth cv_bridge error: %s", e.what());
        }
    }

    void detectionCallback(const vision_pkg::Detection::ConstPtr& msg)
    {
        if (!has_camera_info_ || !has_depth_) return;

        int u = msg->x + msg->width / 2;
        int v = msg->y + msg->height / 2;

        if (u < 0 || u >= depth_image_.cols || v < 0 || v >= depth_image_.rows) return;

        std::vector<uint16_t> valid_depths;
        valid_depths.reserve(25);
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int su = u + dx;
                int sv = v + dy;
                if (su < 0 || su >= depth_image_.cols || sv < 0 || sv >= depth_image_.rows) continue;
                uint16_t d = depth_image_.at<uint16_t>(sv, su);
                if (d > 0 && d <= 10000) valid_depths.push_back(d);
            }
        }
        if (valid_depths.empty()) return;
        std::nth_element(valid_depths.begin(), valid_depths.begin() + valid_depths.size() / 2, valid_depths.end());
        uint16_t depth_mm = valid_depths[valid_depths.size() / 2];

        double z = depth_mm * 0.001;
        double x = (u - cx_) * z / fx_;
        double y = (v - cy_) * z / fy_;

        // --- 单目标 EMA 平滑（向后兼容，只跟踪最高置信度目标） ---
        if (msg->confidence > best_confidence_) {
            best_confidence_ = msg->confidence;

            if (first_pose_) {
                smooth_x_ = x; smooth_y_ = y; smooth_z_ = z;
                first_pose_ = false;
            } else {
                smooth_x_ = smooth_alpha_ * x + (1.0 - smooth_alpha_) * smooth_x_;
                smooth_y_ = smooth_alpha_ * y + (1.0 - smooth_alpha_) * smooth_y_;
                smooth_z_ = smooth_alpha_ * z + (1.0 - smooth_alpha_) * smooth_z_;
            }

            cached_pose_.header.frame_id = "paw_camera_color_optical_frame";
            cached_pose_.pose.position.x = smooth_x_;
            cached_pose_.pose.position.y = smooth_y_;
            cached_pose_.pose.position.z = smooth_z_;
            cached_pose_.pose.orientation.w = 1.0;
            has_pose_ = true;
        }

        if (!reset_timer_.isValid()) {
            reset_timer_ = nh_.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
                best_confidence_ = 0.0f;
            }, true);
        } else {
            reset_timer_.setPeriod(ros::Duration(0.05));
            reset_timer_.start();
        }

        // --- 收集到 pending 列表，run() 里统一发布 ---
        vision_pkg::DetectedObject3D obj;
        obj.class_name = msg->class_name;
        obj.id = msg->id;
        obj.confidence = msg->confidence;
        obj.pose.header.stamp = ros::Time::now();
        obj.pose.header.frame_id = "paw_camera_color_optical_frame";
        obj.pose.pose.position.x = x;
        obj.pose.pose.position.y = y;
        obj.pose.pose.position.z = z;
        obj.pose.pose.orientation.w = 1.0;
        pending_objects_.push_back(obj);
    }

    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher array_pub_;
    ros::Subscriber det_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber info_sub_;
    ros::Timer reset_timer_;

    int publish_rate_;
    bool has_camera_info_;
    bool has_depth_;
    bool has_pose_;
    bool first_pose_;

    double fx_, fy_, cx_, cy_;
    double smooth_alpha_;
    double smooth_x_, smooth_y_, smooth_z_;
    float best_confidence_ = 0.0f;
    cv::Mat depth_image_;
    geometry_msgs::PoseStamped cached_pose_;
    std::vector<vision_pkg::DetectedObject3D> pending_objects_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_pose_node");
    ObjectPoseNode node;
    node.run();
    return 0;
}
