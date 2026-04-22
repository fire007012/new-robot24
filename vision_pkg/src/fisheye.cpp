#include "vision_pkg/fisheye.h"
#include <ros/ros.h>
#include <cmath>

Fisheye::Fisheye() {}

int Fisheye::detectFisheyeRadius(const cv::Mat &img)
{
    cv::Mat gray;
    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img;

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 2);

    cv::Mat mask;
    cv::threshold(blurred, mask, 15, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return std::min(img.rows, img.cols) / 2;

    int best = 0;
    double max_area = 0;
    for (int i = 0; i < (int)contours.size(); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > max_area) { max_area = a; best = i; }
    }

    cv::Point2f center;
    float r;
    cv::minEnclosingCircle(contours[best], center, r);
    return (int)r;
}

cv::Mat Fisheye::cropFisheyeCircle(const cv::Mat &img, int detected_radius)
{
    int h = img.rows, w = img.cols;
    int cx = w / 2, cy = h / 2;
    int half = detected_radius;
    int x1 = std::max(cx - half, 0);
    int y1 = std::max(cy - half, 0);
    int x2 = std::min(cx + half, w);
    int y2 = std::min(cy + half, h);
    int side = std::min(x2 - x1, y2 - y1);
    x1 = cx - side / 2;
    y1 = cy - side / 2;
    return img(cv::Rect(x1, y1, side, side)).clone();
}

void Fisheye::buildUnwarpMaps(int radius, int out_w, int out_h)
{
    const double PI = M_PI;
    front_map1_ = cv::Mat(out_h, out_w, CV_32FC1);
    front_map2_ = cv::Mat(out_h, out_w, CV_32FC1);
    const double source_half_fov = std::max(1.0, static_cast<double>(source_fov_deg_) / 2.0) * PI / 180.0;

    double center = radius;

    for (int py = 0; py < out_h; ++py) {
        double elevation = (0.5 - (double)py / out_h) * PI;
        double cos_elev = cos(elevation);
        double sin_elev = sin(elevation);

        for (int px = 0; px < out_w; ++px) {
            double azimuth = ((double)px / out_w - 0.5) * PI;

            double dx = cos_elev * sin(azimuth);
            double dy = -sin_elev;
            double dz = cos_elev * cos(azimuth);

            double theta = acos(std::max(-1.0, std::min(dz, 1.0)));
            double r = theta / source_half_fov * radius;

            if (r >= radius) {
                front_map1_.at<float>(py, px) = -1;
                front_map2_.at<float>(py, px) = -1;
                continue;
            }

            double phi = atan2(dy, dx);
            front_map1_.at<float>(py, px) = (float)(center + r * cos(phi));
            front_map2_.at<float>(py, px) = (float)(center + r * sin(phi));
        }
    }

    map_radius_ = radius;
    unwarp_maps_ready_ = true;
}

cv::Mat Fisheye::stitch360(const cv::Mat &front_unwarp, const cv::Mat &back_unwarp)
{
    int H = front_unwarp.rows;
    int half_w = front_unwarp.cols;
    int total_w = half_w * 2;
    int blend_w = std::max(total_w / 40, 4);

    cv::Mat pano = cv::Mat::zeros(H, total_w, front_unwarp.type());
    int quarter = total_w / 4;

    front_unwarp.copyTo(pano(cv::Rect(quarter, 0, half_w, H)));

    int back_half = half_w / 2;
    back_unwarp(cv::Rect(half_w / 2, 0, back_half, H))
        .copyTo(pano(cv::Rect(0, 0, quarter, H)));
    back_unwarp(cv::Rect(0, 0, back_half, H))
        .copyTo(pano(cv::Rect(quarter + half_w, 0, quarter, H)));

    for (int y = 0; y < H; ++y) {
        for (int i = 0; i < blend_w; ++i) {
            float a = (float)i / blend_w;
            int c = quarter - blend_w / 2 + i;
            if (c < 0 || c >= total_w) continue;
            cv::Vec3b bg = pano.at<cv::Vec3b>(y, c);
            int fc = c - quarter;
            if (fc >= 0 && fc < half_w) {
                cv::Vec3b fg = front_unwarp.at<cv::Vec3b>(y, fc);
                for (int ch = 0; ch < 3; ++ch)
                    pano.at<cv::Vec3b>(y, c)[ch] =
                        (uchar)((1.0f - a) * bg[ch] + a * fg[ch]);
            }
        }
        for (int i = 0; i < blend_w; ++i) {
            float a = 1.0f - (float)i / blend_w;
            int c = quarter + half_w - blend_w / 2 + i;
            if (c < 0 || c >= total_w) continue;
            cv::Vec3b bg = pano.at<cv::Vec3b>(y, c);
            int fc = c - quarter;
            if (fc >= 0 && fc < half_w) {
                cv::Vec3b fg = front_unwarp.at<cv::Vec3b>(y, fc);
                for (int ch = 0; ch < 3; ++ch)
                    pano.at<cv::Vec3b>(y, c)[ch] =
                        (uchar)(a * fg[ch] + (1.0f - a) * bg[ch]);
            }
        }
    }

    return pano;
}

cv::Mat Fisheye::renderPerspective(const cv::Mat &equirect, float yaw_deg, float pitch_deg,
                                   int out_w, int out_h, float fov_deg)
{
    if (equirect.empty()) return cv::Mat();

    int pano_w = equirect.cols;
    int pano_h = equirect.rows;

    bool need_rebuild = (yaw_deg != cached_yaw_ || pitch_deg != cached_pitch_ ||
                         fov_deg != cached_fov_ || out_w != cached_out_w_ ||
                         out_h != cached_out_h_ || pano_w != cached_pano_w_ ||
                         pano_h != cached_pano_h_);

    if (need_rebuild) {
        const double PI = M_PI;
        double yaw = yaw_deg * PI / 180.0;
        double pitch = pitch_deg * PI / 180.0;
        double fov = fov_deg * PI / 180.0;
        double half_fov = fov / 2.0;

        persp_map1_ = cv::Mat(out_h, out_w, CV_32FC1);
        persp_map2_ = cv::Mat(out_h, out_w, CV_32FC1);

        double f = (out_w / 2.0) / tan(half_fov);

        double cy = cos(yaw), sy = sin(yaw);
        double cp = cos(pitch), sp = sin(pitch);

        for (int py = 0; py < out_h; ++py) {
            for (int px = 0; px < out_w; ++px) {
                double x = px - out_w / 2.0;
                double y = py - out_h / 2.0;
                double z = f;

                double norm = sqrt(x * x + y * y + z * z);
                x /= norm; y /= norm; z /= norm;

                // rotate by pitch (around X)
                double y2 = y * cp - z * sp;
                double z2 = y * sp + z * cp;
                double x2 = x;

                // rotate by yaw (around Y)
                double x3 = x2 * cy + z2 * sy;
                double z3 = -x2 * sy + z2 * cy;
                double y3 = y2;

                // 3D direction → equirectangular coordinates
                double lon = atan2(x3, z3);          // -PI to PI
                double lat = asin(std::max(-1.0, std::min(y3, 1.0)));  // -PI/2 to PI/2

                // equirect pixel coordinates
                double u = (lon / (2.0 * PI) + 0.5) * pano_w;
                double v = (0.5 - lat / PI) * pano_h;

                // wrap horizontally
                if (u < 0) u += pano_w;
                if (u >= pano_w) u -= pano_w;

                persp_map1_.at<float>(py, px) = (float)u;
                persp_map2_.at<float>(py, px) = (float)v;
            }
        }

        cached_yaw_ = yaw_deg;
        cached_pitch_ = pitch_deg;
        cached_fov_ = fov_deg;
        cached_out_w_ = out_w;
        cached_out_h_ = out_h;
        cached_pano_w_ = pano_w;
        cached_pano_h_ = pano_h;
    }

    cv::Mat result;
    cv::remap(equirect, result, persp_map1_, persp_map2_,
              cv::INTER_LINEAR, cv::BORDER_WRAP);
    return result;
}

cv::Mat Fisheye::renderAzimuthal(const cv::Mat &equirect, int out_size)
{
    if (equirect.empty()) return cv::Mat();

    int pano_w = equirect.cols;
    int pano_h = equirect.rows;

    bool need_rebuild = (out_size != cached_azim_size_ ||
                         pano_w != cached_azim_pano_w_ ||
                         pano_h != cached_azim_pano_h_);

    if (need_rebuild) {
        const double PI = M_PI;
        azim_map1_ = cv::Mat(out_size, out_size, CV_32FC1);
        azim_map2_ = cv::Mat(out_size, out_size, CV_32FC1);

        double center = out_size / 2.0;
        double max_r = out_size / 2.0;

        for (int py = 0; py < out_size; ++py) {
            for (int px = 0; px < out_size; ++px) {
                double dx = px - center;
                double dy = py - center;
                double r = sqrt(dx * dx + dy * dy);

                if (r > max_r) {
                    azim_map1_.at<float>(py, px) = -1;
                    azim_map2_.at<float>(py, px) = -1;
                    continue;
                }

                // r maps to colatitude (0 at center = front, max_r = back)
                double colat = (r / max_r) * PI;  // 0 ~ PI

                // angle around circle = azimuth
                double azimuth = atan2(dx, -dy);  // -PI ~ PI, top = forward

                // equirectangular coordinates
                // lon: -PI ~ PI → u: 0 ~ pano_w
                double u = (azimuth / (2.0 * PI) + 0.5) * pano_w;
                // lat: PI/2 ~ -PI/2 → v: 0 ~ pano_h
                // colat 0 = north pole (lat=PI/2), colat PI = south pole (lat=-PI/2)
                double lat = PI / 2.0 - colat;
                double v = (0.5 - lat / PI) * pano_h;

                if (u < 0) u += pano_w;
                if (u >= pano_w) u -= pano_w;

                azim_map1_.at<float>(py, px) = (float)u;
                azim_map2_.at<float>(py, px) = (float)v;
            }
        }

        cached_azim_size_ = out_size;
        cached_azim_pano_w_ = pano_w;
        cached_azim_pano_h_ = pano_h;
    }

    cv::Mat result;
    cv::remap(equirect, result, azim_map1_, azim_map2_,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return result;
}

void Fisheye::processFrames(const cv::Mat &frame1, const cv::Mat &frame2)
{
    if (frame1.empty() || frame2.empty()) {
        ROS_ERROR_THROTTLE(5.0, "Fisheye: input frame empty");
        return;
    }

    if (!unwarp_maps_ready_ || frame1.cols != last_input_w_ || frame1.rows != last_input_h_) {
        detected_radius_ = detectFisheyeRadius(frame1);
        last_input_w_ = frame1.cols;
        last_input_h_ = frame1.rows;
        ROS_INFO("Detected fisheye circle radius: %d (image: %dx%d)",
                 detected_radius_, frame1.cols, frame1.rows);
        unwarp_maps_ready_ = false;
    }

    cv::Mat crop1 = cropFisheyeCircle(frame1, detected_radius_);
    cv::Mat crop2 = cropFisheyeCircle(frame2, detected_radius_);

    int radius = crop1.cols / 2;
    int out_w = radius * 2;
    int out_h = radius * 2;

    if (!unwarp_maps_ready_ || map_radius_ != radius || cached_source_fov_deg_ != source_fov_deg_) {
        buildUnwarpMaps(radius, out_w, out_h);
        cached_source_fov_deg_ = source_fov_deg_;
        ROS_INFO("Fisheye unwarp maps built: radius=%d, output=%dx%d, source_fov=%.1f",
                 radius, out_w, out_h, source_fov_deg_);
    }

    cv::Mat front_unwarp;
    cv::remap(crop1, front_unwarp, front_map1_, front_map2_,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat crop2_rot;
    cv::rotate(crop2, crop2_rot, cv::ROTATE_180);
    cv::Mat back_unwarp;
    cv::remap(crop2_rot, back_unwarp, front_map1_, front_map2_,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat pano = stitch360(front_unwarp, back_unwarp);

    if (m_callback && !pano.empty()) {
        m_callback(pano, pano);
    }
}

