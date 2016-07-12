#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "MemoryLoc.h"
#include "ImageSearch.h"
#include "HTTPServer.h"
#include "Time.h"

#define TOP_K 5

class ImageSearch;
class HTTPServer;

class WordSearch :
    public QObject
{
public:
    WordSearch();
    virtual ~WordSearch();

    void setWords(Words *words);
    void setImageSearch(ImageSearch *imageSearch);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<int> searchWords(rtabmap::SensorData *sensorData, void *context);

private:
    Words *_words;
    ImageSearch *_imageSearch;
};
