#ifndef VISION_PKG_PANORAMA_H
#define VISION_PKG_PANORAMA_H

#include <opencv2/opencv.hpp>
#include <functional>
#include <mutex>
#include <atomic>
#include "vision_pkg/fisheye.h"

class Panorama
{
public:
    using PanoramaReadyCallback = std::function<void(const cv::Mat&, const cv::Mat&)>;

    Panorama();
    ~Panorama();

    void setFisheyeSourceFovDeg(float fov_deg) { fisheye.setSourceFovDeg(fov_deg); }
    void receiveCam1(const cv::Mat &frame);
    void receiveCam2(const cv::Mat &frame);
    void setViewAngle(float yaw_deg, float pitch_deg);
    void resetView();

    void setPanoramaReadyCallback(PanoramaReadyCallback cb) { m_callback = std::move(cb); }

private:
    Fisheye fisheye;
    cv::Mat frame1, frame2;
    std::mutex frameMutex;

    std::atomic<float> view_yaw_{0.0f};
    std::atomic<float> view_pitch_{0.0f};

    PanoramaReadyCallback m_callback;
};

#endif // VISION_PKG_PANORAMA_H

