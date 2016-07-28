#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <memory>
#include <QObject>
#include <QEvent>
#include "adapter/RTABMapDBAdapter.h"
#include "stage/SignatureSearch.h"

class SignatureSearch;
class HTTPServer;

class WordSearch :
    public QObject
{
public:
    WordSearch();
    virtual ~WordSearch();

    void setWords(std::unique_ptr<Words> &&words);
    void setSignatureSearch(SignatureSearch *imageSearch);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<int> searchWords(const rtabmap::SensorData &sensorData) const;

private:
    std::unique_ptr<Words> _words;
    SignatureSearch *_imageSearch;
};
