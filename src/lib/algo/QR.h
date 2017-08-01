#pragma once

#include <opencv2/core/core.hpp>


class FoundItem;
class QR {
public:
  std::vector<FoundItem> QRdetect(const cv::Mat &im);
};