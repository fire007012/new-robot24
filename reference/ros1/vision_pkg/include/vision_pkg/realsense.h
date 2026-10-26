#ifndef VISION_PKG_REALSENSE_H
#define VISION_PKG_REALSENSE_H

#include <string>
#include <atomic>
#include <functional>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief Intel RealSense 深度相机采集类
 *
 * 封装 librealsense2 SDK，支持通过序列号或设备索引选择相机。
 * 同时开启彩色流（1280x720 BGR8）、深度流（Z16）和红外流（Y8），
 * 在独立线程中以 30FPS 采集彩色帧并通过回调输出。
 */
class Realsense
{
public:
    using FrameCallback = std::function<void(const cv::Mat&)>;

    /**
     * @brief 构造函数
     * @param deviceSerial 设备序列号（空字符串则使用 deviceIndex）
     * @param deviceIndex  设备索引（-1 表示不使用索引）
     */
    Realsense(const std::string &deviceSerial, int deviceIndex = -1);
    ~Realsense();

    /** @brief 初始化并启动 RealSense 管线 */
    bool initAndOpenCamera();
    /** @brief 停止管线并释放资源 */
    void closeCamera();

    /** @brief 阻塞式采集循环，需在独立线程中调用 */
    void captureLoop();

    void setFrameCallback(FrameCallback cb) { m_callback = std::move(cb); }

private:
    std::string m_deviceSerial;             // 目标设备序列号
    int m_deviceIndex;                      // 目标设备索引
    std::atomic<bool> m_isCapturing{false}; // 采集状态标志

    rs2::context *m_context = nullptr;      // RealSense 上下文
    rs2::pipeline *m_pipe = nullptr;        // 数据管线
    rs2::device *m_device = nullptr;        // 设备句柄
    rs2::pipeline_profile m_profile;        // 管线配置

    FrameCallback m_callback;               // 帧就绪回调
};

#endif // VISION_PKG_REALSENSE_H
