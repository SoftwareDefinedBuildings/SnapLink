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

    bool init(const std::string &dir);

    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    bool readLabels(const std::string &dir);
    std::vector<std::string> *process(const rtabmap::SensorData *data, const rtabmap::Transform &pose);

private:
    // labels
    std::vector<cv::Point3f> _points;
    std::vector<std::string> _labels;
    HTTPServer *_httpServer;
};

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> &vec);
    bool operator()(const PairType &left, const PairType &right) const;
};
