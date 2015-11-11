#pragma once

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <numeric>

namespace rtabmap {

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> & vec);
    bool operator()(const PairType & left, const PairType & right) const;
};

class RTABMAP_EXP Visibility {

public:
    Visibility();
    virtual ~Visibility();

    bool init(const std::string & labelFolder);
    void process(const SensorData & data, const Transform & pose);

private:
    bool readLabels(const std::string & labelFolder);
    bool isInFrontOfCamera(const cv::Point3f & point, const Transform & P);

private:
    // labels
    std::vector<cv::Point3f> _points;
    std::vector<std::string> _labels;
};

} // namespace rtabmap
