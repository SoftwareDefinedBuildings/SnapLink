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
    const std::map<int, Transform> *optimizedPoses,
    std::string *rejectedMsg,
    int *inliers,
    double *variance) const
{
    std::vector<Signature> oldSigs;
    for (std::vector<int>::const_iterator it = oldIds.begin() ; it != oldIds.end(); it++)
    {
        const Signature *oldSig = getSignature(*it);
        if (oldSig == NULL)
        {
            return Transform();
        }
        // TODO: how much memcopy is here? Maybe cv::Mat covers that?
        oldSigs.push_back(*oldSig);
    }

    const Signature *newSig = NULL;
    newSig = getSignature(newId);
    if (newSig == NULL)
    {
        return Transform();
    }

    return computeGlobalVisualTransform(oldSigs, *newSig, optimizedPoses, rejectedMsg, inliers, variance);
}

Transform MemoryLoc::computeGlobalVisualTransform(
    const std::vector<Signature> &oldSigs,
    const Signature &newSig,
    const std::map<int, Transform> *optimizedPoses,
    std::string *rejectedMsg,
    int *inliersOut,
    double *varianceOut) const
{
    if (oldSigs.size() == 0)
    {
        return Transform();
    }

    Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, pcl::PointXYZ> words3;

    const std::vector<Signature>::const_iterator firstSig = oldSigs.begin();
    const Transform &basePose = getPose(*firstSig, optimizedPoses);

    for (std::vector<Signature>::const_iterator sigIter = oldSigs.begin(); sigIter != oldSigs.end(); sigIter++)
    {
        Transform relativeT = basePose.inverse() * getPose(*sigIter, optimizedPoses);
        const std::multimap<int, pcl::PointXYZ> &sigWords3 = sigIter->getWords3();
        std::multimap<int, pcl::PointXYZ>::const_iterator word3Iter;
        for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
        {
            pcl::PointXYZ point = util3d::transformPoint(word3Iter->second, relativeT);
            //std::multimap<int, pcl::PointXYZ>::iterator it3 = words3.find(word3Iter->first);
            //if (it3 != words3.end())
            //{
            //    std::cout<< "existing point in base frame: " << it3->second << std::endl;
            //    std::cout<< "new point in own frame: " << word3Iter->second << std::endl;
            //    std::cout<< "new point in base frame: " << point << std::endl << std::endl;
            //    std::cout<< "base pose: " << basePose << std::endl;
            //    std::cout<< "own pose: " << sigIter->getPose() << std::endl;
            //}
            words3.insert(std::pair<int, pcl::PointXYZ>(word3Iter->first, point));
        }
    }

    if (!newSig.sensorData().stereoCameraModel().isValid() && (newSig.sensorData().cameraModels().size() != 1 || !newSig.sensorData().cameraModels()[0].isValid()))
    {
        UERROR("Calibrated camera required (multi-cameras not supported).");
        return Transform();
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= _bowMinInliers && (int)newSig.getWords().size() >= _bowMinInliers)
    {
        UASSERT(newSig.sensorData().stereoCameraModel().isValid() || (newSig.sensorData().cameraModels().size() == 1 && newSig.sensorData().cameraModels()[0].isValid()));
        const CameraModel &cameraModel = newSig.sensorData().stereoCameraModel().isValid() ? newSig.sensorData().stereoCameraModel().left() : newSig.sensorData().cameraModels()[0];

        std::vector<int> inliers;
        transform = util3d::estimateMotion3DTo2D(
                        uMultimapToMap(words3),
                        uMultimapToMap(newSig.getWords()),
                        cameraModel, // cameraModel.localTransform has to be the same for all images
                        _bowMinInliers,
                        _bowIterations,
                        _bowPnPReprojError,
                        _bowPnPFlags,
                        Transform::getIdentity(),
                        uMultimapToMap(newSig.getWords3()),
                        &variance,
                        0,
                        &inliers);
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            msg = uFormat("Not enough inliers %d/%d between the old signatures and %d", inliersCount, _bowMinInliers, newSig.id());
            UINFO(msg.c_str());
        }
        else
        {
            transform = transform.inverse();
        }
    }
    else
    {
        msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)", (int)words3.size(), (int)newSig.getWords().size(), _bowMinInliers);
        UINFO(msg.c_str());
    }


    if (!transform.isNull())
    {
        // verify if it is a 180 degree transform, well verify > 90
        float x, y, z, roll, pitch, yaw;
        transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
        if (fabs(roll) > CV_PI / 2 || fabs(pitch) > CV_PI / 2 || fabs(yaw) > CV_PI / 2)
        {
            transform.setNull();
            msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)", roll, pitch, yaw);
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

const Transform MemoryLoc::getPose(const Signature &sig, const std::map<int, Transform> *optimizedPoses) const
{
    Transform pose = sig.getPose();
    if (optimizedPoses)
    {
        const std::map<int, Transform>::const_iterator poseIter = optimizedPoses->find(sig.id());
        if (poseIter != optimizedPoses->end())
        {
            pose = poseIter->second;
        }
    }

    return pose;
}

} // namespace rtabmap
