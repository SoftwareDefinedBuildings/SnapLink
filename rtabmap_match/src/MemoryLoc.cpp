#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>
#include <pcl/io/ply_io.h>
#include <algorithm>
#include <iostream>
#include <fstream>

#include "Utility.h"
#include "MemoryLoc.h"

namespace rtabmap
{

MemoryLoc::MemoryLoc(const ParametersMap &parameters) :
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

    _voxelSize = 0.03f;
    _filteringRadius = 0.1f;
    _filteringMinNeighbors = 5;
    _MLSRadius = 0.1f;
    _MLSpolygonialOrder = 2;
    _MLSUpsamplingMethod = 0; // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
    _MLSUpsamplingRadius = 0.0f;   // SAMPLE_LOCAL_PLANE
    _MLSUpsamplingStep = 0.0f;     // SAMPLE_LOCAL_PLANE
    _MLSPointDensity = 0;            // RANDOM_UNIFORM_DENSITY
    _MLSDilationVoxelSize = 0.04f;  // VOXEL_GRID_DILATION
    _MLSDilationIterations = 0;     // VOXEL_GRID_DILATION
    _normalK = 20;
    _gp3Radius = 0.1f;
    _gp3Mu = 5;
}

Transform MemoryLoc::computeGlobalVisualTransform(
    const std::vector<int> &oldIds,
    int newId,
    std::string *rejectedMsg,
    int *inliers,
    double *variance) const
{
    bool success = true;

    std::vector<Signature> oldSs;
    for (std::vector<int>::const_iterator it = oldIds.begin() ; it != oldIds.end(); it++)
    {
        if (*it)
        {
            const Signature *oldS = getSignature(*it);
            const Transform &pose = oldS->getPose();
            oldSs.push_back(*oldS);
        }
        else
        {
            success = false;
            break;
        }
    }

    const Signature *newS = NULL;
    if (newId)
    {
        newS = getSignature(newId);
    }
    else
    {
        success = false;
    }

    if (success)
    {
        return computeGlobalVisualTransform(oldSs, *newS, rejectedMsg, inliers, variance);
    }
    else
    {
        std::string msg = uFormat("Did not find nodes in oldIds and/or %d", newId);
        if (rejectedMsg)
        {
            *rejectedMsg = msg;
        }
        UWARN(msg.c_str());
    }
    return Transform();
}

Transform MemoryLoc::computeGlobalVisualTransform(
    const std::vector<Signature> &oldSs,
    const Signature &newS,
    std::string *rejectedMsg,
    int *inliersOut,
    double *varianceOut) const
{
    Transform transform;
    std::string msg;
    // Guess transform from visual words

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, pcl::PointXYZ> words3;
    const Transform &basePose = oldSs.begin()->getPose();
    for (std::vector<Signature>::const_iterator it1 = oldSs.begin(); it1 != oldSs.end(); it1++)
    {
        Transform relativeT = basePose.inverse() * it1->getPose();
        std::multimap<int, pcl::PointXYZ>::const_iterator it2;
        for (it2 = it1->getWords3().begin(); it2 != it1->getWords3().end(); it2++)
        {
            pcl::PointXYZ globalPoint = util3d::transformPoint(it2->second, relativeT);
            std::multimap<int, pcl::PointXYZ>::iterator it3 = words3.find(it2->first);
            //if (it3 != words3.end())
            //{
            //    std::cout<< "existing point in base frame: " << it3->second << std::endl;
            //    std::cout<< "new point in own frame: " << it2->second << std::endl;
            //    std::cout<< "new point in base frame: " << globalPoint << std::endl << std::endl;
            //    std::cout<< "base pose: " << basePose << std::endl;
            //    std::cout<< "own pose: " << it1->getPose() << std::endl;
            //}
            words3.insert(std::pair<int, pcl::PointXYZ>(it2->first, globalPoint));
        }
    }

    // PnP
    if (!newS.sensorData().stereoCameraModel().isValid() &&
            (newS.sensorData().cameraModels().size() != 1 ||
             !newS.sensorData().cameraModels()[0].isValid()))
    {
        UERROR("Calibrated camera required (multi-cameras not supported).");
    }
    else
    {
        // 3D to 2D
        if ((int)words3.size() >= _bowMinInliers &&
                (int)newS.getWords().size() >= _bowMinInliers)
        {
            UASSERT(newS.sensorData().stereoCameraModel().isValid() || (newS.sensorData().cameraModels().size() == 1 && newS.sensorData().cameraModels()[0].isValid()));
            const CameraModel &cameraModel = newS.sensorData().stereoCameraModel().isValid() ? newS.sensorData().stereoCameraModel().left() : newS.sensorData().cameraModels()[0];

            std::vector<int> inliersV;
            transform = util3d::estimateMotion3DTo2D(
                            uMultimapToMap(words3),
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
            if (transform.isNull())
            {
                msg = uFormat("Not enough inliers %d/%d between the old signatures and %d",
                              inliersCount, _bowMinInliers, newS.id());
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
                          (int)words3.size(), (int)newS.getWords().size(), _bowMinInliers);
            UINFO(msg.c_str());
        }
    }

    if (!transform.isNull())
    {
        // verify if it is a 180 degree transform, well verify > 90
        float x, y, z, roll, pitch, yaw;
        transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
        if (fabs(roll) > CV_PI / 2 ||
                fabs(pitch) > CV_PI / 2 ||
                fabs(yaw) > CV_PI / 2)
        {
            transform.setNull();
            msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)",
                          roll, pitch, yaw);
            UWARN(msg.c_str());
        }
    }

    // transfer to global frame
    transform = basePose * transform.inverse();

    if (rejectedMsg)
    {
        *rejectedMsg = msg;
    }
    if (inliersOut)
    {
        *inliersOut = inliersCount;
    }
    if (varianceOut)
    {
        *varianceOut = variance;
    }
    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    return transform;
}

} // namespace rtabmap
