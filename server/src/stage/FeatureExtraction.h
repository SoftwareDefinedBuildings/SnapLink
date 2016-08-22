#pragma once

#include <QObject>
#include <opencv2/xfeatures2d.hpp>

class WordSearch;

class FeatureExtraction : public QObject {
public:
  FeatureExtraction();
  virtual ~FeatureExtraction();

  bool init();

  void setWordSearch(WordSearch *wordSearch);

protected:
  virtual bool event(QEvent *event);

private:
  void extractFeatures(const cv::Mat &image,
                       std::vector<cv::KeyPoint> &keyPoints,
                       cv::Mat &descriptors) const;

private:
  WordSearch *_wordSearch;

  cv::Ptr<cv::xfeatures2d::SURF> _detector;
};
