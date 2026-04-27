#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
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
    H265DecoderNode() : nh_("~")
    {
        static std::once_flag gst_init_once;
        std::call_once(gst_init_once, []() {
            gst_init(nullptr, nullptr);
        });

        nh_.param<std::string>("input_topic", input_topic_, "/panorama/panorama_image/h265");
        nh_.param<std::string>("output_topic", output_topic_, "/panorama/panorama_image/h265_decoded");
        nh_.param<int>("queue_size", queue_size_, 1);
        nh_.param<int>("pull_timeout_ms", pull_timeout_ms_, 200);

        pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, queue_size_);
        sub_ = nh_.subscribe(input_topic_, queue_size_, &H265DecoderNode::compressedCallback, this);

        ROS_INFO_STREAM(
            "h265_decoder_node started: input_topic=" << input_topic_
            << " output_topic=" << output_topic_
            << " pull_timeout_ms=" << pull_timeout_ms_);
    }

    ~H265DecoderNode()
    {
        shutdownPipeline();
    }

private:
    void compressedCallback(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        if (pub_.getNumSubscribers() == 0) {
            return;
        }

        if (msg->data.empty()) {
            return;
        }

        if (msg->format.find("h265") == std::string::npos &&
            msg->format.find("hevc") == std::string::npos) {
            ROS_WARN_ONCE("h265 decoder received non-h265 format: %s", msg->format.c_str());
        }

        if (!ensurePipeline()) {
            return;
        }

        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        if (!buffer) {
            ROS_WARN_ONCE("h265 decoder failed to allocate GstBuffer");
            return;
        }

        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_WRITE)) {
            gst_buffer_unref(buffer);
            ROS_WARN_ONCE("h265 decoder failed to map GstBuffer");
            return;
        }

        std::memcpy(map_info.data, msg->data.data(), msg->data.size());
        gst_buffer_unmap(buffer, &map_info);

        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        GST_BUFFER_PTS(buffer) = stamp.toNSec();
        GST_BUFFER_DTS(buffer) = stamp.toNSec();

        GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
        if (flow != GST_FLOW_OK) {
            ROS_WARN_ONCE("h265 decoder push buffer failed: %d", static_cast<int>(flow));
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
            ROS_ERROR_ONCE("h265 decoder failed to create pipeline: %s", error ? error->message : "unknown error");
            if (error) {
                g_error_free(error);
            }
            return false;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!appsrc_ || !appsink_) {
            ROS_ERROR_ONCE("h265 decoder failed to get appsrc/appsink");
            shutdownPipelineLocked();
            return false;
        }

        gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
        gst_app_src_set_max_bytes(GST_APP_SRC(appsrc_), 0);

        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            ROS_ERROR_ONCE("h265 decoder failed to set pipeline to PLAYING");
            shutdownPipelineLocked();
            return false;
        }

        ROS_INFO("h265 decoder pipeline ready");
        return true;
    }

    void publishAvailableFrame(const std_msgs::Header& source_header)
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
            ROS_WARN_ONCE("h265 decoder output buffer is smaller than expected");
            return;
        }

        cv::Mat bgr(height, width, CV_8UC3, const_cast<guint8*>(map_info.data));
        cv_bridge::CvImage out;
        out.header = source_header;
        if (out.header.stamp.isZero()) {
            out.header.stamp = ros::Time::now();
        }
        out.encoding = sensor_msgs::image_encodings::BGR8;
        out.image = bgr.clone();
        pub_.publish(out.toImageMsg());

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

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

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
    ros::init(argc, argv, "h265_decoder_node");
    H265DecoderNode node;
    ros::spin();
    return 0;
}
