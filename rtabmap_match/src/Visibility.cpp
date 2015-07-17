/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#include "rtabmap/utilite/ULogger.h"
#include "Visibility.h"

namespace rtabmap {

Visibility::Visibility()
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
}

} // namespace rtabmap
