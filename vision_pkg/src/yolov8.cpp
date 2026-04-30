#include "vision_pkg/yolov8.h"
#include <ros/ros.h>
#include <algorithm>
#include <cstdio>

Yolov8::Yolov8() {}
Yolov8::~Yolov8() = default;

/**
 * @brief 加载 ONNX 模型到 OpenCV DNN 网络
 *
 * 根据 isCuda 参数选择 CUDA 或 CPU 后端。
 * OpenCV >= 4.6 时禁用 Winograd 卷积以提高数值稳定性。
 */
bool Yolov8::ReadModel(cv::dnn::Net& net, const std::string& netPath, bool isCuda) {
    try {
        net = cv::dnn::readNetFromONNX(netPath);
#if (CV_VERSION_MAJOR*100 + CV_VERSION_MINOR) >= 406
        net.enableWinograd(false);
#endif
    } catch (const std::exception& e) {
        ROS_ERROR("Error reading model: %s", e.what());
        return false;
    }

    if (isCuda) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
        ROS_INFO("Inference device: CPU");
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    return true;
}

/**
 * @brief 执行 YOLOv8 目标检测
 *
 * 流程：LetterBox 预处理 -> 前向推理 -> 解析输出张量 -> NMS 过滤
 *
 * YOLOv8 输出张量形状为 [1, 4+num_classes, num_anchors]，
 * 转置后每行为一个候选框：[cx, cy, w, h, class_scores...]
 * 坐标需通过 letterBox 的缩放参数还原到原图尺度。
 */
bool Yolov8::detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputParams>& output)
{
    cv::Mat blob;
    output.clear();
    cv::Mat netInputImg;
    cv::Vec4d params;  // [ratioX, ratioY, padLeft, padTop]

    // LetterBox 预处理：等比缩放到 640x640 并填充灰边
    letterBox(srcImg, netInputImg, params, cv::Size(_netWidth, _netHeight));
    // 归一化到 [0,1] 并转为 blob
    cv::dnn::blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(_netWidth, _netHeight),
                           cv::Scalar(0, 0, 0), true, false);

    net.setInput(blob);
#if (CV_VERSION_MAJOR*100 + CV_VERSION_MINOR) >= 406
    net.enableWinograd(false);
#endif
    // 前向推理，获取所有输出层的结果
    std::vector<cv::Mat> net_output_img;
    net.forward(net_output_img, net.getUnconnectedOutLayersNames());

    // 转置输出张量：从 [1, 4+classes, anchors] 变为 [anchors, 4+classes]
    cv::Mat output0 = cv::Mat(cv::Size(net_output_img[0].size[2], net_output_img[0].size[1]),
                              CV_32F, (float*)net_output_img[0].data).t();
    int net_width = output0.cols;
    int rows = output0.rows;
    int score_array_length = net_width - 4;  // 前4列是 cx,cy,w,h
    float* pdata = (float*)output0.data;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // 遍历每个候选框
    for (int r = 0; r < rows; ++r) {
        // 取第5列开始的类别分数，找最大值
        cv::Mat scores(1, score_array_length, CV_32FC1, pdata + 4);
        cv::Point classIdPoint;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);
        max_class_score = (float)max_class_score;

        if (max_class_score >= _classThreshold) {
            // 将网络输出坐标还原到原图尺度
            // pdata[0..3] = cx, cy, w, h（在 letterBox 后的坐标系中）
            float x = (pdata[0] - params[2]) / params[0];  // 减去左填充，除以缩放比
            float y = (pdata[1] - params[3]) / params[1];  // 减去上填充，除以缩放比
            float w = pdata[2] / params[0];
            float h = pdata[3] / params[1];
            // 中心坐标转左上角坐标
            int left = std::max(int(x - 0.5 * w + 0.5), 0);
            int top = std::max(int(y - 0.5 * h + 0.5), 0);
            class_ids.push_back(classIdPoint.x);
            confidences.push_back(max_class_score);
            boxes.push_back(cv::Rect(left, top, int(w + 0.5), int(h + 0.5)));
        }
        pdata += net_width;  // 移动到下一个候选框
    }

    // NMS 非极大值抑制，去除重叠框
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, _classThreshold, _nmsThreshold, nms_result);

    // 将 NMS 后的结果裁剪到图像边界内
    cv::Rect holeImgRect(0, 0, srcImg.cols, srcImg.rows);
    for (size_t i = 0; i < nms_result.size(); ++i) {
        int idx = nms_result[i];
        OutputParams result;
        result.id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx] & holeImgRect;  // 与图像边界取交集
        output.push_back(result);
    }
    return !output.empty();
}

/**
 * @brief 在图像上绘制检测结果
 *
 * 绘制检测框、类别名称和置信度，支持水平框、旋转框和实例分割掩码。
 * 最终将标注层与原图以 50% 透明度混合。
 */
cv::Mat Yolov8::drawPred(cv::Mat& img, const std::vector<OutputParams>& result,
                         const std::vector<std::string>& classNames,
                         const std::vector<cv::Scalar>& color)
{
    cv::Mat outputImg = img.clone();
    cv::Mat mask = img.clone();
    bool hasMask = false;
    for (size_t i = 0; i < result.size(); i++) {
        int left = 0, top = 0;
        // 绘制水平检测框
        if (result[i].box.area() > 0) {
            cv::rectangle(outputImg, result[i].box, color[result[i].id], 2, 8);
            left = result[i].box.x;
            top = result[i].box.y;
        }
        // 绘制旋转检测框（OBB 模型）
        if (result[i].rotatedBox.size.width * result[i].rotatedBox.size.height > 0) {
            DrawRotatedBox(outputImg, result[i].rotatedBox, color[result[i].id], 2);
            left = result[i].rotatedBox.center.x;
            top = result[i].rotatedBox.center.y;
        }
        // 绘制实例分割掩码
        if (result[i].boxMask.rows && result[i].boxMask.cols > 0) {
            mask(result[i].box).setTo(color[result[i].id], result[i].boxMask);
            hasMask = true;
        }
        // 绘制类别标签和置信度
        std::string label = classNames[result[i].id] + ":" + std::to_string(result[i].confidence);
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        cv::putText(outputImg, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 1,
                    color[result[i].id], 2);
    }
    // 将掩码层与标注图以 50% 透明度混合
    cv::addWeighted(outputImg, 0.5, mask, 0.5, 0, outputImg);
    if (!hasMask) {
        outputImg = img.clone();
    }

    for (size_t i = 0; i < result.size(); i++) {
        const int colorIndex = color.empty()
            ? 0
            : std::max(0, result[i].id) % static_cast<int>(color.size());
        const cv::Scalar drawColor = color.empty()
            ? cv::Scalar(0, 255, 255)
            : color[colorIndex];

        int left = std::max(0, result[i].box.x);
        int top = std::max(0, result[i].box.y);

        if (result[i].box.area() > 0) {
            const cv::Rect box = result[i].box & cv::Rect(0, 0, outputImg.cols, outputImg.rows);
            if (!box.empty()) {
                cv::rectangle(outputImg, box, drawColor, 3, cv::LINE_AA);
                left = box.x;
                top = box.y;
            }
        }

        if (result[i].rotatedBox.size.width * result[i].rotatedBox.size.height > 0) {
            DrawRotatedBox(outputImg, result[i].rotatedBox, drawColor, 3);
            left = cv::saturate_cast<int>(result[i].rotatedBox.center.x);
            top = cv::saturate_cast<int>(result[i].rotatedBox.center.y);
        }

        const std::string className =
            (result[i].id >= 0 && result[i].id < static_cast<int>(classNames.size()))
                ? classNames[result[i].id]
                : "OBJ";
        char confidence[16];
        snprintf(confidence, sizeof(confidence), "%.2f", result[i].confidence);
        const std::string label = className + ":" + confidence;

        int baseLine = 0;
        const double fontScale = 0.6;
        const int textThickness = 2;
        const cv::Size labelSize = cv::getTextSize(
            label, cv::FONT_HERSHEY_SIMPLEX, fontScale, textThickness, &baseLine);
        const int pad = 4;
        int textY = top - pad;
        if (textY - labelSize.height - pad < 0) {
            textY = std::min(outputImg.rows - pad, top + labelSize.height + pad + 2);
        }
        const int maxTextX = std::max(0, outputImg.cols - labelSize.width - pad * 2);
        const int textX = std::min(left, maxTextX);
        const cv::Point bgTl(textX, std::max(0, textY - labelSize.height - pad));
        const cv::Point bgBr(std::min(outputImg.cols - 1, textX + labelSize.width + pad * 2),
                             std::min(outputImg.rows - 1, textY + baseLine + pad));
        cv::rectangle(outputImg, bgTl, bgBr, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(outputImg, label, cv::Point(textX + pad, textY),
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, drawColor, textThickness, cv::LINE_AA);
    }

    return outputImg;
}

/**
 * @brief LetterBox 预处理
 *
 * 将输入图像等比缩放到目标尺寸，不足部分用灰色 (114,114,114) 填充。
 * 输出缩放参数 params，用于后续将检测坐标还原到原图尺度。
 */
void Yolov8::letterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params,
                       const cv::Size& newShape, bool autoShape, bool scaleFill,
                       bool scaleUp, int stride, const cv::Scalar& color)
{
    cv::Size shape = image.size();
    // 计算缩放比例，取宽高中较小的比例以保持宽高比
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp) r = std::min(r, 1.0f);  // 不允许放大

    float ratio[2]{ r, r };
    int new_un_pad[2] = { (int)std::round((float)shape.width * r),
                          (int)std::round((float)shape.height * r) };

    // 计算需要填充的像素数
    auto dw = (float)(newShape.width - new_un_pad[0]);
    auto dh = (float)(newShape.height - new_un_pad[1]);

    if (autoShape) {
        // 对齐到 stride 的倍数
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    } else if (scaleFill) {
        // 强制拉伸，不保持宽高比
        dw = 0.0f; dh = 0.0f;
        new_un_pad[0] = newShape.width;
        new_un_pad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    // 两侧均匀填充
    dw /= 2.0f; dh /= 2.0f;

    if (shape.width != new_un_pad[0] || shape.height != new_un_pad[1]) {
        cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
    } else {
        outImage = image.clone();
    }

    // 添加边框填充
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    params[0] = ratio[0]; params[1] = ratio[1];
    params[2] = left; params[3] = top;
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}
