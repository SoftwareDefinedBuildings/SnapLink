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
    std::unique_ptr<rtabmap::SensorData> createSensorData(const std::vector<char> &data, double fx, double fy, double cx, double cy) const;

private:
    FeatureExtraction *_feature;
    HTTPServer *_httpServer;
};
