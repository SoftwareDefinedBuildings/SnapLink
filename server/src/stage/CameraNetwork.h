#pragma once

#include <rtabmap/core/Camera.h>
#include <QObject>
#include "stage/FeatureExtraction.h"
#include "stage/HTTPServer.h"

class FeatureExtraction;
class HTTPServer;

class CameraNetwork :
    public QObject
{
public:
    CameraNetwork();
    virtual ~CameraNetwork();

    void setFeatureExtraction(FeatureExtraction *feature);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    // ownership transferred
    std::unique_ptr<rtabmap::SensorData> createSensorData(std::vector<char> *data, double fx, double fy, double cx, double cy);

private:
    FeatureExtraction *_feature;
    HTTPServer *_httpServer;
};
