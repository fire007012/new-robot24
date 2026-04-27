#include "vision_pkg/panorama.h"
#include <ros/ros.h>
#include <algorithm>

Panorama::Panorama()
{
    fisheye.setFramesReadyCallback(
        [this](const cv::Mat &pano, const cv::Mat &) {
            if (pano.empty()) return;

            cv::Mat primary_view;
            if (display_mode_ == "stacked") {
                cv::Mat front_view = fisheye.renderPerspective(
                    pano,
                    0.0f,
                    0.0f,
                    operator_view_width_,
                    operator_view_height_,
                    operator_view_fov_deg_);
                cv::Mat back_view = fisheye.renderPerspective(
                    pano,
                    180.0f,
                    0.0f,
                    operator_view_width_,
                    operator_view_height_,
                    operator_view_fov_deg_);

                if (!front_view.empty() && !back_view.empty()) {
                    primary_view = cv::Mat::zeros(
                        operator_view_height_ * 2,
                        operator_view_width_,
                        front_view.type());
                    front_view.copyTo(primary_view(cv::Rect(0, 0, operator_view_width_, operator_view_height_)));
                    back_view.copyTo(primary_view(cv::Rect(0, operator_view_height_, operator_view_width_, operator_view_height_)));
                }
            } else {
                primary_view = fisheye.renderAzimuthal(pano, 640);
            }

            if (m_callback && !primary_view.empty()) {
                m_callback(primary_view, pano);
            }
        });
}

Panorama::~Panorama() {}

void Panorama::setOperatorViewSize(int width, int height)
{
    operator_view_width_ = std::max(width, 1);
    operator_view_height_ = std::max(height, 1);
}

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

void Panorama::processFrames(const cv::Mat &frame1, const cv::Mat &frame2)
{
    if (frame1.empty() || frame2.empty()) return;

    if (display_mode_ == "cropped_stacked") {
        cv::Mat front_crop;
        cv::Mat back_crop;
        fisheye.extractConfiguredCrops(frame1, frame2, front_crop, back_crop);

        if (!front_crop.empty() && !back_crop.empty()) {
            int canvas_w = std::max(front_crop.cols, back_crop.cols);
            int canvas_h = front_crop.rows + back_crop.rows;
            cv::Mat stacked = cv::Mat::zeros(canvas_h, canvas_w, front_crop.type());

            int front_x = (canvas_w - front_crop.cols) / 2;
            int back_x = (canvas_w - back_crop.cols) / 2;
            front_crop.copyTo(stacked(cv::Rect(front_x, 0, front_crop.cols, front_crop.rows)));
            back_crop.copyTo(stacked(cv::Rect(back_x, front_crop.rows, back_crop.cols, back_crop.rows)));

            if (m_callback) {
                m_callback(stacked, cv::Mat());
            }
        }
        return;
    }

    fisheye.processFrames(frame1, frame2);
}
