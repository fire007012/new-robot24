#ifndef VISION_PKG_FISHEYE_H
#define VISION_PKG_FISHEYE_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

class Fisheye
{
public:
    using FramesReadyCallback = std::function<void(const cv::Mat&, const cv::Mat&)>;

    Fisheye();

    void setSourceFovDeg(float fov_deg) { source_fov_deg_ = fov_deg; }
    void setFramesReadyCallback(FramesReadyCallback cb) { m_callback = std::move(cb); }
    void processFrames(const cv::Mat &frame1, const cv::Mat &frame2);

    // CPU sphere perspective rendering
    cv::Mat renderPerspective(const cv::Mat &equirect, float yaw_deg, float pitch_deg,
                              int out_w = 640, int out_h = 480, float fov_deg = 90.0f);

    // Azimuthal equidistant projection (circular top-down view of the sphere)
    cv::Mat renderAzimuthal(const cv::Mat &equirect, int out_size = 640);

private:
    int detectFisheyeRadius(const cv::Mat &img);
    cv::Mat cropFisheyeCircle(const cv::Mat &img, int detected_radius);
    void buildUnwarpMaps(int radius, int out_w, int out_h);
    cv::Mat stitch360(const cv::Mat &front, const cv::Mat &back);

    // fisheye unwarp maps
    cv::Mat front_map1_, front_map2_;
    int map_radius_ = 0;
    bool unwarp_maps_ready_ = false;
    float source_fov_deg_ = 180.0f;
    float cached_source_fov_deg_ = -1.0f;

    int detected_radius_ = 0;
    int last_input_w_ = 0, last_input_h_ = 0;

    // perspective render maps (cached, rebuilt on view change)
    cv::Mat persp_map1_, persp_map2_;
    float cached_yaw_ = -9999, cached_pitch_ = -9999, cached_fov_ = -9999;
    int cached_out_w_ = 0, cached_out_h_ = 0;
    int cached_pano_w_ = 0, cached_pano_h_ = 0;

    // azimuthal map cache
    cv::Mat azim_map1_, azim_map2_;
    int cached_azim_size_ = 0, cached_azim_pano_w_ = 0, cached_azim_pano_h_ = 0;

    FramesReadyCallback m_callback;
};

#endif // VISION_PKG_FISHEYE_H

