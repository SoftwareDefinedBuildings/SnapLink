#pragma once

#include <opencv2/xfeatures2d.hpp>
#include <QObject>
#include "stage/WordSearch.h"
#include "data/SensorData.h"

class WordSearch;

class FeatureExtraction :
    public QObject
{
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
