#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>

class H265DecoderNode
{
public:
    rclcpp::Node::SharedPtr node() const { return nh_; }
    H265DecoderNode() : nh_(std::make_shared<rclcpp::Node>("h265_decoder_node"))
    {
        static std::once_flag gst_init_once;
        std::call_once(gst_init_once, []() {
            gst_init(nullptr, nullptr);
        });

        input_topic_ = nh_->declare_parameter<std::string>("input_topic", "/panorama/panorama_image/h265");
        output_topic_ = nh_->declare_parameter<std::string>("output_topic", "/panorama/panorama_image/h265_decoded");
        queue_size_ = nh_->declare_parameter<int>("queue_size", 1);
        pull_timeout_ms_ = nh_->declare_parameter<int>("pull_timeout_ms", 200);

        pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(output_topic_, queue_size_);
        sub_ = nh_->create_subscription<sensor_msgs::msg::CompressedImage>(input_topic_, rclcpp::SensorDataQoS(), std::bind(&H265DecoderNode::compressedCallback, this, std::placeholders::_1));

        RCLCPP_INFO_STREAM(rclcpp::get_logger("vision_pkg"), 
            "h265_decoder_node started: input_topic=" << input_topic_
            << " output_topic=" << output_topic_
            << " pull_timeout_ms=" << pull_timeout_ms_);
    }

    ~H265DecoderNode()
    {
        shutdownPipeline();
    }

private:
    void compressedCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
    {
        if (pub_->get_subscription_count() == 0) {
            return;
        }

        if (msg->data.empty()) {
            return;
        }

        if (msg->format.find("h265") == std::string::npos &&
            msg->format.find("hevc") == std::string::npos) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder received non-h265 format: %s", msg->format.c_str());
        }

        if (!ensurePipeline()) {
            return;
        }

        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        if (!buffer) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder failed to allocate GstBuffer");
            return;
        }

        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_WRITE)) {
            gst_buffer_unref(buffer);
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder failed to map GstBuffer");
            return;
        }

        std::memcpy(map_info.data, msg->data.data(), msg->data.size());
        gst_buffer_unmap(buffer, &map_info);

        const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp).nanoseconds()==0 ? nh_->now() : rclcpp::Time(msg->header.stamp);
        GST_BUFFER_PTS(buffer) = stamp.nanoseconds();
        GST_BUFFER_DTS(buffer) = stamp.nanoseconds();

        GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
        if (flow != GST_FLOW_OK) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder push buffer failed: %d", static_cast<int>(flow));
            return;
        }

        publishAvailableFrame(msg->header);
    }

    bool ensurePipeline()
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        if (pipeline_) {
            return true;
        }

        const std::string pipeline_desc =
            "appsrc name=src is-live=true block=true format=time do-timestamp=false "
            "caps=video/x-h265,stream-format=byte-stream,alignment=au "
            "! queue leaky=downstream max-size-buffers=1 "
            "! h265parse "
            "! decodebin "
            "! videoconvert "
            "! video/x-raw,format=BGR "
            "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder failed to create pipeline: %s", error ? error->message : "unknown error");
            if (error) {
                g_error_free(error);
            }
            return false;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!appsrc_ || !appsink_) {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder failed to get appsrc/appsink");
            shutdownPipelineLocked();
            return false;
        }

        gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
        gst_app_src_set_max_bytes(GST_APP_SRC(appsrc_), 0);

        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder failed to set pipeline to PLAYING");
            shutdownPipelineLocked();
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("vision_pkg"), "h265 decoder pipeline ready");
        return true;
    }

    void publishAvailableFrame(const std_msgs::msg::Header& source_header)
    {
        GstSample* sample = gst_app_sink_try_pull_sample(
            GST_APP_SINK(appsink_),
            gst_util_uint64_scale_int(std::max(pull_timeout_ms_, 1), GST_MSECOND, 1));
        if (!sample) {
            return;
        }

        GstCaps* caps = gst_sample_get_caps(sample);
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        if (!caps || !buffer) {
            gst_sample_unref(sample);
            return;
        }

        GstStructure* structure = gst_caps_get_structure(caps, 0);
        int width = 0;
        int height = 0;
        if (!gst_structure_get_int(structure, "width", &width) ||
            !gst_structure_get_int(structure, "height", &height) ||
            width <= 0 || height <= 0) {
            gst_sample_unref(sample);
            return;
        }

        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return;
        }

        const size_t expected_size = static_cast<size_t>(width) * static_cast<size_t>(height) * 3U;
        if (map_info.size < expected_size) {
            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
            RCLCPP_WARN_ONCE(rclcpp::get_logger("vision_pkg"), "h265 decoder output buffer is smaller than expected");
            return;
        }

        cv::Mat bgr(height, width, CV_8UC3, const_cast<guint8*>(map_info.data));
        cv_bridge::CvImage out;
        out.header = source_header;
        if (rclcpp::Time(out.header.stamp).nanoseconds()==0) {
            out.header.stamp = nh_->now();
        }
        out.encoding = sensor_msgs::image_encodings::BGR8;
        out.image = bgr.clone();
        pub_->publish(*out.toImageMsg());

        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
    }

    void shutdownPipeline()
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        shutdownPipelineLocked();
    }

    void shutdownPipelineLocked()
    {
        if (appsrc_) {
            gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
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
    }

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    std::string input_topic_;
    std::string output_topic_;
    int queue_size_{1};
    int pull_timeout_ms_{200};

    std::mutex pipeline_mutex_;
    GstElement* pipeline_{nullptr};
    GstElement* appsrc_{nullptr};
    GstElement* appsink_{nullptr};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    H265DecoderNode node;
    rclcpp::spin(node.node());
    return 0;
}
