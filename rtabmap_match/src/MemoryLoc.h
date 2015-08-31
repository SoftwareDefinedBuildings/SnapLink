#ifndef MEMORYLOC_H_
#define MEMORYLOC_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/Features2d.h"
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include "rtabmap/utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_types.h>

#include "rtabmap/core/Memory.h"

namespace rtabmap {

class Signature;

class RTABMAP_EXP MemoryLoc : public Memory
{
public:
    MemoryLoc(const ParametersMap & parameters = ParametersMap());

    Transform computeGlobalVisualTransform(const std::vector<int> & oldIds, int newId, std::string * rejectedMsg = 0, int * inliers = 0, double * variance = 0) const;
    Transform computeGlobalVisualTransform(const std::vector<Signature> & oldSs, const Signature & newS, std::string * rejectedMsg = 0, int * inliers = 0, double * variance = 0) const;

private:
    int _bowMinInliers;
    int _bowIterations;
    int _bowRefineIterations;
    double _bowPnPReprojError;
    int _bowPnPFlags;
};

} // namespace rtabmap

#endif /* MEMORYLOC_H_ */
