#pragma once

#include "data/SensorData.h"
#include "stage/WordSearch.h"
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
  void extractFeatures(SensorData &sensorData) const;

private:
  WordSearch *_wordSearch;

  cv::Ptr<cv::xfeatures2d::SURF> _detector;
};
