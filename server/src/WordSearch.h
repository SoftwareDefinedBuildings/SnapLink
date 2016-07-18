#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "MemoryLoc.h"
#include "SignatureSearch.h"
#include "HTTPServer.h"
#include "Time.h"

class SignatureSearch;
class HTTPServer;

class WordSearch :
    public QObject
{
public:
    WordSearch();
    virtual ~WordSearch();

    void setWords(const Words *words);
    void setSignatureSearch(SignatureSearch *imageSearch);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<int> searchWords(rtabmap::SensorData *sensorData, void *context);

private:
    const Words *_words;
    SignatureSearch *_imageSearch;
};
