#ifndef VISION_PKG_QR_DETECTOR_H
#define VISION_PKG_QR_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <string>
#include <vector>

namespace vision_pkg
{

class QrDetector
{
public:
    struct Result
    {
        std::vector<cv::Point> corners;
        std::string text;
    };

    QrDetector(bool enabled,
               double eps_x,
               double eps_y,
               bool require_decoded_text);
    ~QrDetector() = default;

    QrDetector(const QrDetector&) = delete;
    QrDetector& operator=(const QrDetector&) = delete;

    bool enabled() const;
    std::vector<Result> detect(const cv::Mat& src);
    void draw(cv::Mat& dst, const std::vector<Result>& results) const;

private:
    static std::string shortenText(const std::string& text, size_t max_len = 48);
    static bool extractCorners(const cv::Mat& points, int index, std::vector<cv::Point>& corners);
    static bool isValidCorners(const std::vector<cv::Point>& corners, const cv::Size& size);
    static bool isDuplicateResult(const std::vector<Result>& results,
                                  const std::vector<cv::Point>& corners);
    static void drawResult(cv::Mat& img, const Result& result);

    bool enabled_{false};
    bool require_decoded_text_{true};
    cv::QRCodeDetector detector_;
};

}  // namespace vision_pkg

#endif  // VISION_PKG_QR_DETECTOR_H
