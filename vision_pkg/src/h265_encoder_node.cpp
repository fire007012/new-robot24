#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstring>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>

class H265EncoderNode
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    H265EncoderNode() : nh_(std::make_shared<rclcpp::Node>("h265_encoder_node"))
    {
        static std::once_flag gst_init_once;
        std::call_once(gst_init_once, []() {
            gst_init(nullptr, nullptr);
        });

        input_topic_ = nh_->declare_parameter<std::string>("input_topic", "/panorama/panorama_image");
        output_topic_ = nh_->declare_parameter<std::string>("output_topic", "/panorama/panorama_image/h265");
        queue_size_ = nh_->declare_parameter<int>("queue_size", 1);
        bitrate_kbps_ = nh_->declare_parameter<int>("bitrate_kbps", 1200);
        fps_ = nh_->declare_parameter<int>("fps", 15);
        gop_ = nh_->declare_parameter<int>("gop", 15);
        max_input_fps_ = nh_->declare_parameter<double>("max_input_fps", 15.0);

        pub_ = nh_->create_publisher<sensor_msgs::msg::CompressedImage>(output_topic_, queue_size_);
        sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(input_topic_, rclcpp::SensorDataQoS(), std::bind(&H265EncoderNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO_STREAM(rclcpp::get_logger("vision_pkg"), 
            "h265_encoder_node started: input_topic=" << input_topic_
            << " output_topic=" << output_topic_
            << " bitrate_kbps=" << bitrate_kbps_
            << " fps=" << fps_
            << " gop=" << gop_
            << " max_input_fps=" << max_input_fps_);
    }

    ~H265EncoderNode()
    {
        shutdownPipeline();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (pub_->get_subscription_count() == 0) {
            return;
        }

        const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp).nanoseconds()==0 ? nh_->now() : rclcpp::Time(msg->header.stamp);
        if (max_input_fps_ > 0.0 && !(last_input_stamp_.nanoseconds()==0)) {
            const double min_dt = 1.0 / max_input_fps_;
            if ((stamp - last_input_stamp_).seconds() < min_dt) {
                return;
            }
        }
        last_input_stamp_ = stamp;

        cv_bridge::CvImageConstPtr image;
        try {
            image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 cv_bridge error: %s", e.what());
            return;
        }

        if (image->image.empty()) {
            return;
        }

        if (!ensurePipeline(image->image.cols, image->image.rows)) {
            return;
        }

        const uint64_t pts_ns = stamp.nanoseconds();
        {
            std::lock_guard<std::mutex> lock(meta_mutex_);
            pending_frame_ids_[pts_ns] = msg->header.frame_id;
            while (pending_frame_ids_.size() > 256) {
                pending_frame_ids_.erase(pending_frame_ids_.begin());
            }
        }

        const size_t bytes = image->image.total() * image->image.elemSize();
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
        if (!buffer) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 failed to allocate GstBuffer");
            return;
        }

        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_WRITE)) {
            gst_buffer_unref(buffer);
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 failed to map GstBuffer");
            return;
        }

        std::memcpy(map_info.data, image->image.data, bytes);
        gst_buffer_unmap(buffer, &map_info);

        GST_BUFFER_PTS(buffer) = pts_ns;
        GST_BUFFER_DTS(buffer) = pts_ns;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, std::max(fps_, 1));

        GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
        if (flow != GST_FLOW_OK) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 push buffer failed: %d", static_cast<int>(flow));
        }
    }

    bool ensurePipeline(int width, int height)
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        if (pipeline_ && width == pipeline_width_ && height == pipeline_height_) {
            return true;
        }

        shutdownPipelineLocked();

        pipeline_width_ = width;
        pipeline_height_ = height;

        const std::string pipeline_desc =
            "appsrc name=src is-live=true block=true format=time do-timestamp=false "
            "caps=video/x-raw,format=BGR,width=" + std::to_string(width) +
            ",height=" + std::to_string(height) +
            ",framerate=" + std::to_string(std::max(fps_, 1)) + "/1 "
            "! queue leaky=downstream max-size-buffers=1 "
            "! videoconvert "
            "! video/x-raw,format=I420 "
            "! x265enc tune=zerolatency speed-preset=ultrafast bitrate=" + std::to_string(std::max(bitrate_kbps_, 100)) +
            " key-int-max=" + std::to_string(std::max(gop_, 1)) +
            " bframes=0 "
            "! h265parse config-interval=-1 "
            "! video/x-h265,stream-format=byte-stream,alignment=au "
            "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("vision_pkg"), "h265 failed to create pipeline: %s", error ? error->message : "unknown error");
            if (error) {
                g_error_free(error);
            }
            return false;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!appsrc_ || !appsink_) {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("vision_pkg"), "h265 failed to get appsrc/appsink");
            shutdownPipelineLocked();
            return false;
        }

        gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
        gst_app_src_set_max_bytes(GST_APP_SRC(appsrc_), 0);

        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("vision_pkg"), "h265 failed to set pipeline to PLAYING");
            shutdownPipelineLocked();
            return false;
        }

        running_.store(true);
        output_thread_ = std::thread(&H265EncoderNode::outputLoop, this);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("vision_pkg"), "h265 pipeline ready: " << width << "x" << height);
        return true;
    }

    void outputLoop()
    {
        while (running_.load()) {
            GstSample* sample = gst_app_sink_try_pull_sample(
                GST_APP_SINK(appsink_), gst_util_uint64_scale_int(200, GST_MSECOND, 1));
            if (!sample) {
                continue;
            }

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            if (!buffer) {
                gst_sample_unref(sample);
                continue;
            }

            GstMapInfo map_info;
            if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            sensor_msgs::msg::CompressedImage msg;
            msg.format = "h265";

            const uint64_t pts_ns =
                GST_CLOCK_TIME_IS_VALID(GST_BUFFER_PTS(buffer))
                    ? static_cast<uint64_t>(GST_BUFFER_PTS(buffer))
                    : 0ULL;
            if (pts_ns != 0ULL) {
                msg.header.stamp = rclcpp::Time(pts_ns, RCL_ROS_TIME);
                std::lock_guard<std::mutex> lock(meta_mutex_);
                auto it = pending_frame_ids_.find(pts_ns);
                if (it != pending_frame_ids_.end()) {
                    msg.header.frame_id = it->second;
                    pending_frame_ids_.erase(it);
                }
            } else {
                msg.header.stamp = nh_->now();
            }

            msg.data.assign(map_info.data, map_info.data + map_info.size);
            pub_->publish(msg);

            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
        }
    }

    void shutdownPipeline()
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        shutdownPipelineLocked();
    }

    void shutdownPipelineLocked()
    {
        running_.store(false);

        if (appsrc_) {
            gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
        }

        if (output_thread_.joinable()) {
            output_thread_.join();
        }

        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
        }

        if (appsrc_) {
            gst_object_unref(appsrc_);
            appsrc_ = nullptr;
        }
        if (appsink_) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }
        if (pipeline_) {
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }

        pipeline_width_ = 0;
        pipeline_height_ = 0;
        std::lock_guard<std::mutex> meta_lock(meta_mutex_);
        pending_frame_ids_.clear();
    }

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;

    std::string input_topic_;
    std::string output_topic_;
    int queue_size_{1};
    int bitrate_kbps_{1200};
    int fps_{15};
    int gop_{15};
    double max_input_fps_{15.0};
    rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};

    std::mutex pipeline_mutex_;
    GstElement* pipeline_{nullptr};
    GstElement* appsrc_{nullptr};
    GstElement* appsink_{nullptr};
    int pipeline_width_{0};
    int pipeline_height_{0};
    std::atomic<bool> running_{false};
    std::thread output_thread_;

    std::mutex meta_mutex_;
    std::map<uint64_t, std::string> pending_frame_ids_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    H265EncoderNode node;
    rclcpp::spin(node.node());
    return 0;
}
