/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#ifndef VISIBILITY_H_
#define VISIBILITY_H_

#include <iostream>
#include <fstream>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>

namespace rtabmap {

class RTABMAP_EXP Visibility {
public:
    Visibility(const CameraModel & model);
    virtual ~Visibility();

    bool init(const std::string & cloudFile, const std::string & labelFolder);
    void process(SensorData data, Transform pose);

private:
    // HDR

    // for test purpose
    std::vector<cv::Point2f> Generate2DPoints();
    std::vector<cv::Point3f> Generate3DPoints();

private:
    CameraModel _model;
    // ccpointcloud
    // labels
};

} // namespace rtabmap


#endif /* VISIBILITY_H_ */
