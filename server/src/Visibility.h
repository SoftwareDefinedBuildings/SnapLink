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

    bool init(std::vector<std::string> &dbfiles);

    void setMemory(MemoryLoc *memory);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    bool processLabels(std::vector<std::string> &dbfiles);
    std::vector<std::string> *process(int dbId, const rtabmap::SensorData *data, const rtabmap::Transform &pose) const;
    bool getPoint3World(int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld);

private:
    // labels
    std::vector< std::vector<cv::Point3f> > _points;
    std::vector< std::vector<std::string> > _labels;
    HTTPServer *_httpServer;
    MemoryLoc *_memory;
};

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> &vec);
    bool operator()(const PairType &left, const PairType &right) const;
};
