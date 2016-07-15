#pragma once

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <numeric>
#include "HTTPServer.h"

class HTTPServer;

class Visibility :
    public QObject
{
public:
    Visibility();
    virtual ~Visibility();

    void setMemory(RTABMapDBAdapter *memory);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<std::string> *process(int dbId, const rtabmap::SensorData *data, const rtabmap::Transform &pose) const;

private:
    // labels
    std::vector< std::vector< std::pair<cv::Point3f, std::string> > > _labels;
    HTTPServer *_httpServer;
    RTABMapDBAdapter *_memory;
};

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> &vec);
    bool operator()(const PairType &left, const PairType &right) const;
};
