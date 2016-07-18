#pragma once

#include <rtabmap/core/Camera.h>
#include <QObject>
#include "FeatureExtraction.h"
#include "HTTPServer.h"

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
    rtabmap::SensorData *createSensorData(std::vector<unsigned char> *data, int width, int height, double fx, double fy, double cx, double cy);

private:
    FeatureExtraction *_feature;
    HTTPServer *_httpServer;
};
