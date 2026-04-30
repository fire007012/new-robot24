#ifndef VISION_PKG_YOLOV8_H
#define VISION_PKG_YOLOV8_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include "yolov8_utils.h"

/**
 * @brief YOLOv8 目标检测封装类
 *
 * 通过 OpenCV DNN 模块加载 ONNX 模型，支持 CPU 和 CUDA 推理。
 * 用于检测 15 类危险品标识。
 */
class Yolov8
{
public:
    Yolov8();
    ~Yolov8();

    /**
     * @brief 加载 ONNX 模型
     * @param net     OpenCV DNN 网络对象（输出）
     * @param netPath ONNX 模型文件路径
     * @param isCuda  是否使用 CUDA 加速
     * @return 加载成功返回 true
     */
    bool ReadModel(cv::dnn::Net& net, const std::string& netPath, bool isCuda = false);

    /**
     * @brief 对输入图像执行目标检测
     * @param srcImg 输入图像（BGR格式）
     * @param net    已加载的 DNN 网络
     * @param output 检测结果列表（输出）
     * @return 检测到目标返回 true
     */
    bool detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputParams>& output);

    /**
     * @brief 在图像上绘制检测结果（检测框 + 类别标签 + 置信度）
     * @param img        原始图像
     * @param result     检测结果列表
     * @param classNames 类别名称列表
     * @param color      每个类别对应的颜色
     * @return 绘制后的图像副本
     */
    cv::Mat drawPred(cv::Mat& img, const std::vector<OutputParams>& result,
                     const std::vector<std::string>& classNames,
                     const std::vector<cv::Scalar>& color);

    // 15 类危险品标识名称
    std::vector<std::string> className = {
        "FLAMMABLE GAS", "FUELOIL", "ORGANICPEROXIDE", "OXIDIZER",
        "DANGEROUS", "FLAMMABLE SOLID", "EXPLOSIVES", "OXYGEN",
        "POISON", "NON-FLAMMABLE GAS", "COMBUSTIBLE", "INHALATION HAZARD",
        "RADIOACTIVE", "BLASTING AGENTS", "CORROSIVE", "BUTTON", "VICTIM"
    };

private:
    /**
     * @brief LetterBox 预处理：等比缩放并填充灰边，保持宽高比
     * @param image     输入图像
     * @param outImage  输出图像（已缩放+填充）
     * @param params    缩放参数 [ratioX, ratioY, padLeft, padTop]，用于后续坐标还原
     * @param newShape  目标尺寸，默认 640x640
     * @param autoShape 是否自动对齐到 stride 的倍数
     * @param scaleFill 是否强制拉伸填满（不保持宽高比）
     * @param scaleUp   是否允许放大（false 则只缩小不放大）
     * @param stride    对齐步长，默认 32
     * @param color     填充颜色，默认灰色 (114,114,114)
     */
    void letterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params,
                   const cv::Size& newShape = cv::Size(640, 640),
                   bool autoShape = false, bool scaleFill = false,
                   bool scaleUp = true, int stride = 32,
                   const cv::Scalar& color = cv::Scalar(114, 114, 114));

    int _netWidth = 640;          // 网络输入宽度
    int _netHeight = 640;         // 网络输入高度
    float _classThreshold = 0.7;  // 类别置信度阈值（提高到0.5）
    float _nmsThreshold = 0.45;   // NMS（非极大值抑制）IoU 阈值
};

#endif // VISION_PKG_YOLOV8_H
