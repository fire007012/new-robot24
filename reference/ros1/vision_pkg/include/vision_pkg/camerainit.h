#ifndef VISION_PKG_CAMERAINIT_H
#define VISION_PKG_CAMERAINIT_H

#include <string>
#include <atomic>
#include <functional>
#include <opencv2/opencv.hpp>

/**
 * @brief 通用 USB 相机采集类（基于 V4L2）
 *
 * 通过 OpenCV VideoCapture 打开指定设备路径的摄像头，
 * 在独立线程中以 30FPS 循环采集帧，通过回调函数输出。
 */
class CameraInit
{
public:
    using FrameCallback = std::function<void(const cv::Mat&)>;

    /**
     * @brief 构造函数
     * @param devicePath V4L2 设备路径，如 "/dev/video0"
     */
    explicit CameraInit(const std::string& devicePath);
    ~CameraInit();

    /** @brief 打开摄像头设备 */
    bool initAndOpenCamera();
    /** @brief 关闭摄像头并释放资源 */
    void closeCamera();
    /** @brief 查询摄像头是否已打开 */
    bool isCameraOpen();

    /** @brief 阻塞式采集循环，需在独立线程中调用，以 30FPS 采集并回调 */
    void captureLoop();

    void setFrameCallback(FrameCallback cb) { m_callback = std::move(cb); }

private:
    std::string m_devicePath;            // 设备路径
    cv::VideoCapture m_cap;              // OpenCV 视频采集对象
    std::atomic<bool> m_isCapturing{false}; // 采集状态标志
    FrameCallback m_callback;            // 帧就绪回调
};

#endif // VISION_PKG_CAMERAINIT_H
