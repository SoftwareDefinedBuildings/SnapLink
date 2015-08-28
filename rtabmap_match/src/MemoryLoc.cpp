#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UProcessInfo.h>
#include <rtabmap/utilite/UMath.h>

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/VWDictionary.h"
#include <rtabmap/core/EpipolarGeometry.h>
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/Graph.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "MemoryLoc.h"

namespace rtabmap {

MemoryLoc::MemoryLoc(const ParametersMap & parameters) :
    Memory(parameters),
    _bowMinInliers(Parameters::defaultLccBowMinInliers()),
    _bowIterations(Parameters::defaultLccBowIterations()),
    _bowRefineIterations(Parameters::defaultLccBowRefineIterations()),
    _bowPnPReprojError(Parameters::defaultLccBowPnPReprojError()),
    _bowPnPFlags(Parameters::defaultLccBowPnPFlags())
{
    Parameters::parse(parameters, Parameters::kLccBowMinInliers(), _bowMinInliers);
    Parameters::parse(parameters, Parameters::kLccBowIterations(), _bowIterations);
    Parameters::parse(parameters, Parameters::kLccBowRefineIterations(), _bowRefineIterations);
    Parameters::parse(parameters, Parameters::kLccBowPnPReprojError(), _bowPnPReprojError);
    Parameters::parse(parameters, Parameters::kLccBowPnPFlags(), _bowPnPFlags);

    UASSERT_MSG(_bowMinInliers >= 1, uFormat("value=%d", _bowMinInliers).c_str());
    UASSERT_MSG(_bowIterations > 0, uFormat("value=%d", _bowIterations).c_str());
}

Transform MemoryLoc::computeGlobalVisualTransform(
        int oldId,
        int newId,
        std::string * rejectedMsg,
        int * inliers,
        double * variance) const
{
    const Signature * oldS = this->getSignature(oldId);
    const Signature * newS = this->getSignature(newId);

    Transform transform;

    if(oldS && newId)
    {
        return computeGlobalVisualTransform(*oldS, *newS, rejectedMsg, inliers, variance);
    }
    else
    {
        std::string msg = uFormat("Did not find nodes %d and/or %d", oldId, newId);
        if(rejectedMsg)
        {
            *rejectedMsg = msg;
        }
        UWARN(msg.c_str());
    }
    return Transform();
}

Transform MemoryLoc::computeGlobalVisualTransform(
        const Signature & oldS,
        const Signature & newS,
        std::string * rejectedMsg,
        int * inliersOut,
        double * varianceOut) const
{
    Transform transform;
    std::string msg;
    // Guess transform from visual words

    int inliersCount= 0;
    double variance = 1.0;

    // PnP
    if(!newS.sensorData().stereoCameraModel().isValid() &&
       (newS.sensorData().cameraModels().size() != 1 ||
        !newS.sensorData().cameraModels()[0].isValid()))
    {
        UERROR("Calibrated camera required (multi-cameras not supported).");
    }
    else
    {
        // 3D to 2D
        if((int)oldS.getWords3().size() >= _bowMinInliers &&
           (int)newS.getWords().size() >= _bowMinInliers)
        {
            UASSERT(newS.sensorData().stereoCameraModel().isValid() || (newS.sensorData().cameraModels().size() == 1 && newS.sensorData().cameraModels()[0].isValid()));
            const CameraModel & cameraModel = newS.sensorData().stereoCameraModel().isValid()?newS.sensorData().stereoCameraModel().left():newS.sensorData().cameraModels()[0];

            std::vector<int> inliersV;
            transform = util3d::estimateMotion3DTo2D(
                    uMultimapToMap(oldS.getWords3()),
                    uMultimapToMap(newS.getWords()),
                    cameraModel,
                    _bowMinInliers,
                    _bowIterations,
                    _bowPnPReprojError,
                    _bowPnPFlags,
                    Transform::getIdentity(),
                    uMultimapToMap(newS.getWords3()),
                    &variance,
                    0,
                    &inliersV);
            inliersCount = (int)inliersV.size();
            if(transform.isNull())
            {
                msg = uFormat("Not enough inliers %d/%d between %d and %d",
                        inliersCount, _bowMinInliers, oldS.id(), newS.id());
                UINFO(msg.c_str());
            }
            else
            {
                transform = transform.inverse();
            }
        }
        else
        {
            msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)",
                    (int)oldS.getWords3().size(), (int)newS.getWords().size(), _bowMinInliers);
            UINFO(msg.c_str());
        }
    }

    if(!transform.isNull())
    {
        // verify if it is a 180 degree transform, well verify > 90
        float x,y,z, roll,pitch,yaw;
        transform.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
        if(fabs(roll) > CV_PI/2 ||
           fabs(pitch) > CV_PI/2 ||
           fabs(yaw) > CV_PI/2)
        {
            transform.setNull();
            msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)",
                    roll, pitch, yaw);
            UWARN(msg.c_str());
        }
    }

    if(rejectedMsg)
    {
        *rejectedMsg = msg;
    }
    if(inliersOut)
    {
        *inliersOut = inliersCount;
    }
    if(varianceOut)
    {
        *varianceOut = variance;
    }
    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    return transform;
}

} // namespace rtabmap
