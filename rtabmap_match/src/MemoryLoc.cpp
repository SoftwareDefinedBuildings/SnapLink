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
    _minInliers(rtabmap::Parameters::defaultVisMinInliers()),
    _iterations(rtabmap::Parameters::defaultVisIterations()),
    _pnpRefineIterations(rtabmap::Parameters::defaultVisPnPRefineIterations()),
    _pnpReprojError(rtabmap::Parameters::defaultVisPnPReprojError()),
    _pnpFlags(rtabmap::Parameters::defaultVisPnPFlags())
{
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisMinInliers(), _minInliers);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisIterations(), _iterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPRefineIterations(), _pnpRefineIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPReprojError(), _pnpReprojError);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPFlags(), _pnpFlags);

    UASSERT_MSG(_minInliers >= 1, uFormat("value=%d", _minInliers).c_str());
    UASSERT_MSG(_iterations > 0, uFormat("value=%d", _iterations).c_str());
}

rtabmap::Transform MemoryLoc::computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId) const
{
    std::vector<const rtabmap::Signature *> oldSigs;
    for (std::vector<int>::const_iterator it = oldIds.begin() ; it != oldIds.end(); it++)
    {
        const rtabmap::Signature *oldSig = getSignature(*it);
        if (oldSig == NULL)
        {
            return rtabmap::Transform();
        }
        oldSigs.push_back(oldSig);
    }

    const rtabmap::Signature *newSig = getSignature(newId);
    if (newSig == NULL)
    {
        return rtabmap::Transform();
    }

    return computeGlobalVisualTransform(oldSigs, newSig);
}

rtabmap::Transform MemoryLoc::computeGlobalVisualTransform(const std::vector<const rtabmap::Signature *> &oldSigs, const rtabmap::Signature *newSig) const
{
    if (oldSigs.size() == 0 || newSig == NULL)
    {
        return rtabmap::Transform();
    }

    rtabmap::Transform transform;
    // TODO test to use the world coordiante as basePose
    //rtabmap::Transform transform3;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words;
    //std::multimap<int, cv::Point3f> words3;

    const std::vector<const rtabmap::Signature *>::const_iterator firstSig = oldSigs.begin();
    const rtabmap::Transform &basePose = (*firstSig)->getPose();

    for (std::vector<const rtabmap::Signature *>::const_iterator sigIter = oldSigs.begin(); sigIter != oldSigs.end(); sigIter++)
    {
        rtabmap::Transform relativeT = basePose.inverse() * (*sigIter)->getPose();
        //rtabmap::Transform pose = (*sigIter)->getPose();
        const std::multimap<int, cv::Point3f> &sigWords3 = (*sigIter)->getWords3();
        std::multimap<int, cv::Point3f>::const_iterator word3Iter;
        for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
        {
            cv::Point3f point = rtabmap::util3d::transformPoint(word3Iter->second, relativeT);
            //cv::Point3f point3 = rtabmap::util3d::transformPoint(word3Iter->second, pose);
            words.insert(std::pair<int, cv::Point3f>(word3Iter->first, point));
            //words3.insert(std::pair<int, cv::Point3f>(word3Iter->first, point3));
        }
    }

    // 3D to 2D (PnP)
    if ((int)words.size() >= _minInliers && (int)newSig->getWords().size() >= _minInliers)
    {
        const rtabmap::CameraModel &cameraModel = newSig->sensorData().cameraModels()[0];

        std::vector<int> matches;
        std::vector<int> inliers;
        transform = rtabmap::util3d::estimateMotion3DTo2D(
                        uMultimapToMapUnique(words),
                        uMultimapToMapUnique(newSig->getWords()),
                        cameraModel, // TODO: cameraModel.localTransform has to be the same for all images
                        _minInliers,
                        _iterations,
                        _pnpReprojError,
                        _pnpFlags,
                        _pnpRefineIterations,
                        rtabmap::Transform::getIdentity(),
                        uMultimapToMapUnique(newSig->getWords3()),
                        &variance,
                        &matches,
                        &inliers);
        //transform3 = rtabmap::util3d::estimateMotion3DTo2D(
        //                uMultimapToMapUnique(words3),
        //                uMultimapToMapUnique(newSig->getWords()),
        //                cameraModel, // TODO: cameraModel.localTransform has to be the same for all images
        //                _minInliers,
        //                _iterations,
        //                _pnpReprojError,
        //                _pnpFlags,
        //                _pnpRefineIterations,
        //                rtabmap::Transform::getIdentity(),
        //                uMultimapToMapUnique(newSig->getWords3()),
        //                &variance,
        //                &matches,
        //                &inliers);
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            msg = uFormat("Not enough inliers %d/%d between the old signatures and %d", inliersCount, _minInliers, newSig->id());
            UINFO(msg.c_str());
        }
        else
        {
            transform = transform.inverse();
            //transform3 = transform3.inverse();
        }
    }
    else
    {
        msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)", (int)words.size(), (int)newSig->getWords().size(), _minInliers);
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
    //transform3 = transform3.inverse();

    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    //UDEBUG("transform3=%s", transform3.prettyPrint().c_str());
    return transform;
}

