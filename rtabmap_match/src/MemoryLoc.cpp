#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
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

MemoryLoc::MemoryLoc(const rtabmap::ParametersMap &parameters) :
    rtabmap::Memory(parameters),
    _bowMinInliers(rtabmap::Parameters::defaultLccBowMinInliers()),
    _bowIterations(rtabmap::Parameters::defaultLccBowIterations()),
    _bowRefineIterations(rtabmap::Parameters::defaultLccBowRefineIterations()),
    _bowPnPReprojError(rtabmap::Parameters::defaultLccBowPnPReprojError()),
    _bowPnPFlags(rtabmap::Parameters::defaultLccBowPnPFlags())
{
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kLccBowMinInliers(), _bowMinInliers);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kLccBowIterations(), _bowIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kLccBowRefineIterations(), _bowRefineIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kLccBowPnPReprojError(), _bowPnPReprojError);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kLccBowPnPFlags(), _bowPnPFlags);

    UASSERT_MSG(_bowMinInliers >= 1, uFormat("value=%d", _bowMinInliers).c_str());
    UASSERT_MSG(_bowIterations > 0, uFormat("value=%d", _bowIterations).c_str());
}

rtabmap::Transform MemoryLoc::computeGlobalVisualTransform(
    const std::vector<int> &oldIds,
    int newId,
    const std::map<int, rtabmap::Transform> *optimizedPoses,
    std::string *rejectedMsg,
    int *inliers,
    double *variance) const
{
    std::vector<rtabmap::Signature> oldSigs;
    for (std::vector<int>::const_iterator it = oldIds.begin() ; it != oldIds.end(); it++)
    {
        const rtabmap::Signature *oldSig = getSignature(*it);
        if (oldSig == NULL)
        {
            return rtabmap::Transform();
        }
        // TODO: how much memcopy is here? Maybe cv::Mat covers that?
        oldSigs.push_back(*oldSig);
    }

    const rtabmap::Signature *newSig = NULL;
    newSig = getSignature(newId);
    if (newSig == NULL)
    {
        return rtabmap::Transform();
    }

    return computeGlobalVisualTransform(oldSigs, *newSig, optimizedPoses, rejectedMsg, inliers, variance);
}

rtabmap::Transform MemoryLoc::computeGlobalVisualTransform(
    const std::vector<rtabmap::Signature> &oldSigs,
    const rtabmap::Signature &newSig,
    const std::map<int, rtabmap::Transform> *optimizedPoses,
    std::string *rejectedMsg,
    int *inliersOut,
    double *varianceOut) const
{
    if (oldSigs.size() == 0)
    {
        return rtabmap::Transform();
    }

    rtabmap::Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, pcl::PointXYZ> words3;

    const std::vector<rtabmap::Signature>::const_iterator firstSig = oldSigs.begin();
    const rtabmap::Transform &basePose = getPose(*firstSig, optimizedPoses);

    for (std::vector<rtabmap::Signature>::const_iterator sigIter = oldSigs.begin(); sigIter != oldSigs.end(); sigIter++)
    {
        rtabmap::Transform relativeT = basePose.inverse() * getPose(*sigIter, optimizedPoses);
        const std::multimap<int, pcl::PointXYZ> &sigWords3 = sigIter->getWords3();
        std::multimap<int, pcl::PointXYZ>::const_iterator word3Iter;
        for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
        {
            pcl::PointXYZ point = rtabmap::util3d::transformPoint(word3Iter->second, relativeT);
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
        return rtabmap::Transform();
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= _bowMinInliers && (int)newSig.getWords().size() >= _bowMinInliers)
    {
        UASSERT(newSig.sensorData().stereoCameraModel().isValid() || (newSig.sensorData().cameraModels().size() == 1 && newSig.sensorData().cameraModels()[0].isValid()));
        const rtabmap::CameraModel &cameraModel = newSig.sensorData().stereoCameraModel().isValid() ? newSig.sensorData().stereoCameraModel().left() : newSig.sensorData().cameraModels()[0];

        std::vector<int> inliers;
        transform = rtabmap::util3d::estimateMotion3DTo2D(
                        uMultimapToMap(words3),
                        uMultimapToMap(newSig.getWords()),
                        cameraModel, // cameraModel.localTransform has to be the same for all images
                        _bowMinInliers,
                        _bowIterations,
                        _bowPnPReprojError,
                        _bowPnPFlags,
                        rtabmap::Transform::getIdentity(),
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

const rtabmap::Transform MemoryLoc::getPose(const rtabmap::Signature &sig, const std::map<int, rtabmap::Transform> *optimizedPoses) const
{
    rtabmap::Transform pose = sig.getPose();
    if (optimizedPoses)
    {
        const std::map<int, rtabmap::Transform>::const_iterator poseIter = optimizedPoses->find(sig.id());
        if (poseIter != optimizedPoses->end())
        {
            pose = poseIter->second;
        }
    }

    return pose;
}
