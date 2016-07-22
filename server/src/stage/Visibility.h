#pragma once

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <numeric>
#include <memory>
#include "stage/HTTPServer.h"

class HTTPServer;

class Visibility :
    public QObject
{
public:
    Visibility();
    virtual ~Visibility();

    void setLabels(std::unique_ptr<Labels> &&labels);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<std::string> process(int dbId, const rtabmap::SensorData &sensorData, const rtabmap::Transform &pose) const;

private:
    std::unique_ptr<Labels> _labels;
    HTTPServer *_httpServer;
};

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> &vec);
    bool operator()(const PairType &left, const PairType &right) const;
};
