/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#ifndef VISIBILITY_H_
#define VISIBILITY_H_

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>

#include <numeric>
#include <iostream>
#include <fstream>

namespace rtabmap {

struct CompareMeanDist
{
    typedef std::pair< std::string, std::vector<double> > PairType;

    static double meanDist(const std::vector<double> & vec);
    bool operator()(const PairType & left, const PairType & right) const;
};

class RTABMAP_EXP Visibility {

public:
    Visibility(const CameraModel & model);
    virtual ~Visibility();

    bool init(const std::string & labelFolder);
    void process(const SensorData & data, const Transform & pose);

private:
    bool readLabels(const std::string & labelFolder);
    bool isInFrontOfCamera(const cv::Point3f & point, const Transform & P);

private:
    CameraModel _model;
    // we don't need the whole pointcloud
    
    // labels
    std::vector<cv::Point3f> _points;
    std::vector<std::string> _labels;
    
    std::ofstream resultFile;
};

} // namespace rtabmap


#endif /* VISIBILITY_H_ */
