#ifndef VISION_PKG_PANORAMA_H
#define VISION_PKG_PANORAMA_H

#include <opencv2/opencv.hpp>
#include <functional>
#include <atomic>
#include <string>
#include "vision_pkg/fisheye.h"

class Panorama
{
public:
    using PanoramaReadyCallback = std::function<void(const cv::Mat&, const cv::Mat&)>;

    Panorama();
    ~Panorama();

    void setFisheyeSourceFovDeg(float fov_deg) { fisheye.setSourceFovDeg(fov_deg); }
    void setFrontRotate180(bool enabled) { fisheye.setFrontRotate180(enabled); }
    void setBackRotate180(bool enabled) { fisheye.setBackRotate180(enabled); }
    void setFrontCropRect(int ref_width, int ref_height, int x, int y, int width, int height)
    {
        fisheye.setFrontCropRect(ref_width, ref_height, x, y, width, height);
    }
    void setBackCropRect(int ref_width, int ref_height, int x, int y, int width, int height)
    {
        fisheye.setBackCropRect(ref_width, ref_height, x, y, width, height);
    }
    void setDisplayMode(const std::string &mode) { display_mode_ = mode; }
    void setOperatorViewFovDeg(float fov_deg) { operator_view_fov_deg_ = fov_deg; }
    void setOperatorViewSize(int width, int height);
    void processFrames(const cv::Mat &frame1, const cv::Mat &frame2);
    void setViewAngle(float yaw_deg, float pitch_deg);
    void resetView();

    void setPanoramaReadyCallback(PanoramaReadyCallback cb) { m_callback = std::move(cb); }

private:
    Fisheye fisheye;

    std::atomic<float> view_yaw_{0.0f};
    std::atomic<float> view_pitch_{0.0f};
    std::string display_mode_ = "azimuthal";
    float operator_view_fov_deg_ = 120.0f;
    int operator_view_width_ = 640;
    int operator_view_height_ = 360;

    PanoramaReadyCallback m_callback;
};

#endif // VISION_PKG_PANORAMA_H
