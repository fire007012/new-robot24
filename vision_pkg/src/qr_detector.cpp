#include "vision_pkg/qr_detector.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace vision_pkg
{

QrDetector::QrDetector(bool enabled,
                       double eps_x,
                       double eps_y,
                       bool require_decoded_text)
    : enabled_(enabled),
      require_decoded_text_(require_decoded_text)
{
    detector_.setEpsX(eps_x);
    detector_.setEpsY(eps_y);
}

bool QrDetector::enabled() const
{
    return enabled_;
}

void QrDetector::draw(cv::Mat& dst, const std::vector<Result>& results) const
{
    if (!enabled_ || dst.empty()) {
        return;
    }

    for (const auto& result : results) {
        drawResult(dst, result);
    }
}

std::string QrDetector::shortenText(const std::string& text, size_t max_len)
{
    if (text.size() <= max_len) return text;
    if (max_len <= 3) return text.substr(0, max_len);
    return text.substr(0, max_len - 3) + "...";
}

bool QrDetector::extractCorners(const cv::Mat& points, int index, std::vector<cv::Point>& corners)
{
    corners.clear();
    if (points.empty() || points.channels() != 2) return false;

    if (points.type() == CV_32FC2) {
        if (points.rows > index && points.cols >= 4) {
            for (int i = 0; i < 4; ++i) {
                const cv::Vec2f p = points.at<cv::Vec2f>(index, i);
                corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
            }
            return true;
        }
        if (index == 0 && points.rows >= 4 && points.cols == 1) {
            for (int i = 0; i < 4; ++i) {
                const cv::Vec2f p = points.at<cv::Vec2f>(i, 0);
                corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
            }
            return true;
        }
        if (index == 0 && points.rows == 1 && points.cols >= 4) {
            for (int i = 0; i < 4; ++i) {
                const cv::Vec2f p = points.at<cv::Vec2f>(0, i);
                corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
            }
            return true;
        }
    } else if (points.type() == CV_64FC2) {
        if (points.rows > index && points.cols >= 4) {
            for (int i = 0; i < 4; ++i) {
                const cv::Vec2d p = points.at<cv::Vec2d>(index, i);
                corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
            }
            return true;
        }
        if (index == 0 && points.rows >= 4 && points.cols == 1) {
            for (int i = 0; i < 4; ++i) {
                const cv::Vec2d p = points.at<cv::Vec2d>(i, 0);
                corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
            }
            return true;
        }
        if (index == 0 && points.rows == 1 && points.cols >= 4) {
            for (int i = 0; i < 4; ++i) {
                const cv::Vec2d p = points.at<cv::Vec2d>(0, i);
                corners.emplace_back(cvRound(p[0]), cvRound(p[1]));
            }
            return true;
        }
    }

    return false;
}

bool QrDetector::isValidCorners(const std::vector<cv::Point>& corners, const cv::Size& size)
{
    if (corners.size() != 4 || size.empty()) return false;

    const cv::Rect image_rect(0, 0, size.width, size.height);
    std::vector<cv::Point> clipped;
    clipped.reserve(4);
    for (const auto& p : corners) {
        if (!image_rect.contains(p)) return false;
        clipped.push_back(p);
    }

    const double area = std::abs(cv::contourArea(clipped));
    return area >= 80.0;
}

bool QrDetector::isDuplicateResult(const std::vector<Result>& results,
                                   const std::vector<cv::Point>& corners)
{
    const cv::Rect box = cv::boundingRect(corners);
    if (box.empty()) return true;

    for (const auto& result : results) {
        const cv::Rect other = cv::boundingRect(result.corners);
        const cv::Rect overlap = box & other;
        const double overlap_area = static_cast<double>(overlap.area());
        const double min_area = static_cast<double>(std::max(1, std::min(box.area(), other.area())));
        if (overlap_area / min_area > 0.65) {
            return true;
        }
    }
    return false;
}

void QrDetector::drawResult(cv::Mat& img, const Result& result)
{
    if (result.corners.size() != 4) return;

    std::vector<std::vector<cv::Point>> contour(1, result.corners);
    cv::polylines(img, contour, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    const std::string text = "QR: " + shortenText(result.text);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);

    int min_x = img.cols - 1;
    int min_y = img.rows - 1;
    for (const auto& p : result.corners) {
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
    }

    const int text_x = std::max(0, min_x);
    const int text_y = std::max(text_size.height + 8, min_y - 8);
    const int box_x2 = std::min(img.cols - 1, text_x + text_size.width + 8);
    const int box_y1 = std::max(0, text_y - text_size.height - 6);
    const int box_y2 = std::min(img.rows - 1, text_y + baseline + 2);

    cv::rectangle(img, cv::Point(text_x, box_y1), cv::Point(box_x2, box_y2),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, text, cv::Point(text_x + 4, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

std::vector<QrDetector::Result> QrDetector::detect(const cv::Mat& src)
{
    std::vector<Result> results;
    if (!enabled_ || src.empty()) return results;

    cv::Mat gray;
    if (src.channels() == 1) {
        gray = src;
    } else {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<std::string> decoded_info;
    cv::Mat points;
    auto append_results = [&](const cv::Mat& detected_points,
                              const std::vector<std::string>& decoded_texts) {
        for (int i = 0; i < static_cast<int>(decoded_texts.size()); ++i) {
            const std::string& text = decoded_texts[static_cast<size_t>(i)];
            if (require_decoded_text_ && text.empty()) {
                continue;
            }

            std::vector<cv::Point> corners;
            if (!extractCorners(detected_points, i, corners) ||
                !isValidCorners(corners, gray.size()) ||
                isDuplicateResult(results, corners)) {
                continue;
            }

            results.push_back(Result{corners, text});
        }
    };

    try {
        if (detector_.detectAndDecodeMulti(gray, decoded_info, points)) {
            append_results(points, decoded_info);
        }
    } catch (const cv::Exception&) {
        return {};
    }

    return results;
}

}  // namespace vision_pkg
