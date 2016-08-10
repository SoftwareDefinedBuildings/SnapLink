#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Features2d.h>
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

    bool init(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

    void setWordSearch(WordSearch *wordSearch);

protected:
    virtual bool event(QEvent *event);

private:
    void extractFeatures(SensorData &sensorData) const;

private:
    WordSearch *_wordSearch;

    rtabmap::Feature2D *_feature2D;
};
