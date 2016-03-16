#pragma once

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <numeric>

#include "HTTPServer.h"

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> &vec);
    bool operator()(const PairType &left, const PairType &right) const;
};

class HTTPServer;

class Visibility :
    public QObject
{

public:
    Visibility();
    virtual ~Visibility();

    HTTPServer *_httpserver;

    bool init(const std::string &labelFolder);
    std::vector<std::string> process(const rtabmap::SensorData &data, const rtabmap::Transform &pose);

protected:
    virtual bool event(QEvent *event);

private:
    bool readLabels(const std::string &labelFolder);

private:
    // labels
    std::vector<cv::Point3f> _points;
    std::vector<std::string> _labels;
};
