#include "vision_pkg/panorama.h"
#include <ros/ros.h>

Panorama::Panorama()
{
    fisheye.setFramesReadyCallback(
        [this](const cv::Mat &pano, const cv::Mat &) {
            if (pano.empty()) return;

            cv::Mat azimuthal =fisheye.renderAzimuthal(
                pano, 640);

            if (m_callback && !azimuthal.empty()) {
                m_callback(azimuthal, pano);
            }
        });
}

Panorama::~Panorama() {}

void Panorama::setViewAngle(float yaw_deg, float pitch_deg)
{
    view_yaw_.store(yaw_deg);
    view_pitch_.store(pitch_deg);
}

void Panorama::resetView()
{
    view_yaw_.store(0.0f);
    view_pitch_.store(0.0f);
}

void Panorama::receiveCam1(const cv::Mat &frame)
{
    if (frame.empty()) return;

    cv::Mat bgr;
    if (frame.channels() == 3)
        bgr = frame.clone();
    else
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);

    std::lock_guard<std::mutex> lock(frameMutex);
    frame1 = bgr;
    if (!frame2.empty()) {
        fisheye.processFrames(frame1, frame2);
    }
}

void Panorama::receiveCam2(const cv::Mat &frame)
{
    if (frame.empty()) return;

    cv::Mat bgr;
    if (frame.channels() == 3)
        bgr = frame.clone();
    else
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);

    std::lock_guard<std::mutex> lock(frameMutex);
    frame2 = bgr;
    if (!frame1.empty()) {
        fisheye.processFrames(frame1, frame2);
    }
}

