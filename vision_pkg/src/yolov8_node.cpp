#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <vision_pkg/msg/detection.hpp>
#include "vision_pkg/yolov8.h"

/**
 * @brief YOLOv8 检测 ROS 节点
 *
 * 订阅图像话题，对每帧执行 YOLOv8 推理，
 * 发布每个检测目标的 Detection 消息和标注后的图像。
 *
 * 参数：
 *   ~model_path  - ONNX 模型路径（默认 "model.onnx"）
 *   ~image_topic - 输入图像话题（默认 "/camera/forward/image_raw"）
 *   ~use_cuda    - 是否使用 CUDA（默认 false）
 */
class Yolov8Node
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    Yolov8Node() : nh_(std::make_shared<rclcpp::Node>("yolov8_node"))
    {
        // 读取参数
        std::string model_path, image_topic;
        bool use_cuda;
        model_path = nh_->declare_parameter<std::string>("model_path", "model.onnx");
        image_topic = nh_->declare_parameter<std::string>("image_topic", "/camera/forward/image_raw");
        use_cuda = nh_->declare_parameter<bool>("use_cuda", false);

        // 加载 ONNX 模型
        if (!yolo_.ReadModel(net_, model_path, use_cuda)) {
            RCLCPP_ERROR(rclcpp::get_logger("vision_pkg"), "Failed to load YOLO model: %s", model_path.c_str());
            return;
        }

        // 创建发布者和订阅者
        det_pub_ = nh_->create_publisher<vision_pkg::msg::Detection>("detections", 10);
        img_pub_ = image_transport::create_publisher(nh_.get(), "detection_image");
        sub_ = image_transport::create_subscription(nh_.get(), image_topic, std::bind(&Yolov8Node::imageCallback, this, std::placeholders::_1), "raw", rmw_qos_profile_sensor_data);

        // 为每个类别生成随机颜色（用于绘制检测框）
        for (size_t i = 0; i < yolo_.className.size(); i++) {
            colors_.push_back(cv::Scalar(rand() % 256, rand() % 256, rand() % 256));
        }

        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "yolov8_node started, subscribing to %s", image_topic.c_str());
    }

private:
    /**
     * @brief 图像回调：执行检测并发布结果
     *
     * 对每帧图像执行 YOLOv8 推理，将每个检测目标作为独立的
     * Detection 消息发布，同时发布绘制了检测框的标注图像。
     */
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::vector<OutputParams> results;

        if (yolo_.detect(frame, net_, results)) {
            // 逐个发布检测结果消息
            for (const auto& r : results) {
                vision_pkg::msg::Detection det;
                det.header=msg->header;
                det.id = r.id;
                det.confidence = r.confidence;
                det.x = r.box.x;
                det.y = r.box.y;
                det.width = r.box.width;
                det.height = r.box.height;
                if (r.id >= 0 && r.id < (int)yolo_.className.size())
                    det.class_name = yolo_.className[r.id];
                det_pub_->publish(det);
            }

            // 发布标注图像
            cv::Mat drawn = yolo_.drawPred(frame, results, yolo_.className, colors_);
            auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", drawn).toImageMsg();
            img_pub_.publish(out_msg);
        }
    }

    rclcpp::Node::SharedPtr nh_;
    
    image_transport::Publisher img_pub_;    // 标注图像发布者
    image_transport::Subscriber sub_;       // 输入图像订阅者
    rclcpp::Publisher<vision_pkg::msg::Detection>::SharedPtr det_pub_;                // Detection 消息发布者
    Yolov8 yolo_;                           // YOLOv8 检测器实例
    cv::dnn::Net net_;                      // DNN 网络对象
    std::vector<cv::Scalar> colors_;        // 每个类别的绘制颜色
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    Yolov8Node node;
    rclcpp::spin(node.node());
    return 0;
}
