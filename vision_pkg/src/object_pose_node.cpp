#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <vector>
#include <vision_pkg/msg/detection.hpp>
#include <vision_pkg/msg/detected_object3_d.hpp>
#include <vision_pkg/msg/detected_object3_d_array.hpp>

class ObjectPoseNode
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    ObjectPoseNode() : nh_(std::make_shared<rclcpp::Node>("object_pose_node")), has_camera_info_(false), has_depth_(false), has_pose_(false), first_pose_(true)
    {
        std::string detection_topic, depth_topic, camera_info_topic;
        detection_topic = nh_->declare_parameter<std::string>("detection_topic", "/yolov8/detections");
        depth_topic = nh_->declare_parameter<std::string>("depth_topic", "/paw_camera/depth/image_rect_raw");
        camera_info_topic = nh_->declare_parameter<std::string>("camera_info_topic", "/paw_camera/color/camera_info");
        publish_rate_ = nh_->declare_parameter<int>("publish_rate", 20);
        smooth_alpha_ = nh_->declare_parameter<double>("smooth_alpha", 0.3);

        pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_object_pose", 10);
        array_pub_ = nh_->create_publisher<vision_pkg::msg::DetectedObject3DArray>("detected_objects_3d", 10);

        det_sub_ = nh_->create_subscription<vision_pkg::msg::Detection>(detection_topic, rclcpp::SensorDataQoS(), std::bind(&ObjectPoseNode::detectionCallback, this, std::placeholders::_1));
        depth_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(depth_topic, rclcpp::SensorDataQoS(), std::bind(&ObjectPoseNode::depthCallback, this, std::placeholders::_1));
        info_sub_ = nh_->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic, rclcpp::SensorDataQoS(), std::bind(&ObjectPoseNode::cameraInfoCallback, this, std::placeholders::_1));

        RCLCPP_INFO_STREAM(rclcpp::get_logger("vision_pkg"), 
            "object_pose_node started: detection_topic=" << detection_topic
            << " depth_topic=" << depth_topic
            << " camera_info_topic=" << camera_info_topic
            << " publish_rate=" << publish_rate_
            << " smooth_alpha=" << smooth_alpha_);
    }

    void run()
    {
        rclcpp::WallRate rate(publish_rate_);
        while (rclcpp::ok()) {
            rclcpp::spin_some(nh_);
            if (has_pose_ && (nh_->now()-rclcpp::Time(cached_pose_.header.stamp)).seconds()<.3) {
                pose_pub_->publish(cached_pose_);
            }
            if (!pending_objects_.empty()) {
                vision_pkg::msg::DetectedObject3DArray arr;
                arr.header=pending_objects_.front().pose.header;
                
                arr.objects = std::move(pending_objects_);
                array_pub_->publish(arr);
                pending_objects_.clear();
            }
            rate.sleep();
        }
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        if (has_camera_info_) return;

        // K matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        has_camera_info_ = true;

        info_sub_.reset();
        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            auto source=cv_bridge::toCvCopy(msg)->image;
            if(msg->encoding==sensor_msgs::image_encodings::TYPE_16UC1)source.convertTo(depth_image_,CV_32F,.001);
            else if(msg->encoding==sensor_msgs::image_encodings::TYPE_32FC1)depth_image_=source;
            else throw cv_bridge::Exception("depth must be 16UC1 (mm) or 32FC1 (m)");
            depth_stamp_=msg->header.stamp;
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "depth cv_bridge error: %s", e.what());
        }
    }

    void detectionCallback(const vision_pkg::msg::Detection::ConstSharedPtr& msg)
    {
        if (!has_camera_info_ || !has_depth_ || fx_<=0 || fy_<=0) return;
        const auto stamp=rclcpp::Time(msg->header.stamp);
        if(stamp.nanoseconds()==0 || std::abs((stamp-depth_stamp_).seconds())>.1 || (nh_->now()-stamp).seconds()>.3)return;

        int u = msg->x + msg->width / 2;
        int v = msg->y + msg->height / 2;

        if (u < 0 || u >= depth_image_.cols || v < 0 || v >= depth_image_.rows) return;

        std::vector<float> valid_depths;
        valid_depths.reserve(25);
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int su = u + dx;
                int sv = v + dy;
                if (su < 0 || su >= depth_image_.cols || sv < 0 || sv >= depth_image_.rows) continue;
                float d = depth_image_.at<float>(sv, su);
                if (std::isfinite(d) && d > 0 && d <= 10) valid_depths.push_back(d);
            }
        }
        if (valid_depths.empty()) return;
        std::nth_element(valid_depths.begin(), valid_depths.begin() + valid_depths.size() / 2, valid_depths.end());
        double depth_m = valid_depths[valid_depths.size() / 2];

        double z = depth_m;
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

            cached_pose_.header = msg->header;
            cached_pose_.pose.position.x = smooth_x_;
            cached_pose_.pose.position.y = smooth_y_;
            cached_pose_.pose.position.z = smooth_z_;
            cached_pose_.pose.orientation.w = 1.0;
            has_pose_ = true;
        }

        if (!reset_timer_) {
            reset_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
                best_confidence_ = 0.0f; reset_timer_->cancel();
            });
        } else reset_timer_->reset();

        // --- 收集到 pending 列表，run() 里统一发布 ---
        vision_pkg::msg::DetectedObject3D obj;
        obj.class_name = msg->class_name;
        obj.id = msg->id;
        obj.confidence = msg->confidence;
        obj.pose.header = msg->header;
        
        obj.pose.pose.position.x = x;
        obj.pose.pose.position.y = y;
        obj.pose.pose.position.z = z;
        obj.pose.pose.orientation.w = 1.0;
        pending_objects_.push_back(obj);
    }

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<vision_pkg::msg::DetectedObject3DArray>::SharedPtr array_pub_;
    rclcpp::Subscription<vision_pkg::msg::Detection>::SharedPtr det_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::TimerBase::SharedPtr reset_timer_;

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
    rclcpp::Time depth_stamp_{0,0,RCL_ROS_TIME};
    geometry_msgs::msg::PoseStamped cached_pose_;
    std::vector<vision_pkg::msg::DetectedObject3D> pending_objects_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ObjectPoseNode node;
    node.run();
    return 0;
}
