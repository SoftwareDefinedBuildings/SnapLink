/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#include "opencv2/core/core.hpp"

#include "rtabmap/utilite/ULogger.h"
#include "Visibility.h"

namespace rtabmap {

Visibility::Visibility(const CameraModel & model):
    _model(model)
{
    UDEBUG("");
}

Visibility::~Visibility()
{
    UDEBUG("");
}

bool Visibility::init(const std::string & cloudFile, const std::string & labelFolder)
{
    return true;
}

void Visibility::process(SensorData data, Transform pose)
{
    UDEBUG("processing transform = %s", pose.prettyPrint().c_str());

   // cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);

}

} // namespace rtabmap
