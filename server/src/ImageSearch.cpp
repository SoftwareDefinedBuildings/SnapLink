#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/EpipolarGeometry.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Rtabmap.h>
#include <QCoreApplication>

#include "ImageSearch.h"
#include "WordEvent.h"
#include "LocationEvent.h"
#include "FailureEvent.h"
#include "Signature.h"

ImageSearch::ImageSearch() :
    _memory(NULL),
    _vis(NULL),
    _httpServer(NULL),
    _minInliers(rtabmap::Parameters::defaultVisMinInliers()),
    _iterations(rtabmap::Parameters::defaultVisIterations()),
    _pnpRefineIterations(rtabmap::Parameters::defaultVisPnPRefineIterations()),
    _pnpReprojError(rtabmap::Parameters::defaultVisPnPReprojError()),
    _pnpFlags(rtabmap::Parameters::defaultVisPnPFlags())
{
}

ImageSearch::~ImageSearch()
{
    _memory = NULL;
    _vis = NULL;
    _httpServer = NULL;
}

bool ImageSearch::init(const rtabmap::ParametersMap &parameters)
{
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisMinInliers(), _minInliers);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisIterations(), _iterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPRefineIterations(), _pnpRefineIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPReprojError(), _pnpReprojError);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPFlags(), _pnpFlags);

    return true;
}

void ImageSearch::setMemory(MemoryLoc *memory)
{
    _memory = memory;
}

void ImageSearch::setVisibility(Visibility *vis)
{
    _vis = vis;
}

void ImageSearch::setHTTPServer(HTTPServer *httpServer)
{
    _httpServer = httpServer;
}

bool ImageSearch::event(QEvent *event)
{
    if (event->type() == WordEvent::type())
    {
        WordEvent *wordEvent = static_cast<WordEvent *>(event);
        rtabmap::Transform pose;
        int dbId;
        bool success = localize(wordEvent->wordIds(), wordEvent->sensorData(), &pose, &dbId, wordEvent->conInfo());
        // a null pose notify that loc could not be computed
        if (success)
        {
            QCoreApplication::postEvent(_vis, new LocationEvent(dbId, wordEvent->sensorData(), pose, wordEvent->conInfo()));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(wordEvent->conInfo()));
        }
        return true;
    }
    return QObject::event(event);
}

bool ImageSearch::localize(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, rtabmap::Transform *pose, int *dbId, void *context)
{
    ConnectionInfo *con_info = (ConnectionInfo *) context;

    UASSERT(!sensorData->imageRaw().empty());

    const rtabmap::CameraModel &cameraModel = sensorData->cameraModels()[0];

    con_info->time.search_start = getTime();
    std::vector<int> topIds = _memory->findKNearestSignatures(wordIds, TOP_K);
    con_info->time.search += getTime() - con_info->time.search_start; // end of find closest match
    int topSigId = topIds[0];
    const Signature *topSig = _memory->getSignatures().at(topSigId);

    // TODO: compare interatively until success
    UDEBUG("topSigId: %d", topSigId);
    con_info->time.pnp_start = getTime();
    *pose = computeGlobalVisualTransform(wordIds, sensorData, cameraModel, topSigId);
    con_info->time.pnp += getTime() - con_info->time.pnp_start;
    *dbId = topSig->getDbId();

    if (!pose->isNull())
    {
        UDEBUG("global transform = %s", pose->prettyPrint().c_str());
    }
    else
    {
        UWARN("transform is null, using pose of the closest image");
        *pose = topSig->getPose();
    }

    UINFO("output transform = %s using image %d in database %d", pose->prettyPrint().c_str(), topSigId, topSig->getDbId());

    return !pose->isNull();
}

rtabmap::Transform ImageSearch::computeGlobalVisualTransform(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, const rtabmap::CameraModel &cameraModel, int oldSigId) const
{
    rtabmap::Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words3;

    const Signature *oldSig = _memory->getSignatures().at(oldSigId);
    const rtabmap::Transform &oldSigPose = oldSig->getPose();

    const std::multimap<int, cv::Point3f> &sigWords3 = oldSig->getWords3D();
    std::multimap<int, cv::Point3f>::const_iterator word3Iter;
    for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
    {
        cv::Point3f point3 = rtabmap::util3d::transformPoint(word3Iter->second, oldSigPose);
        words3.insert(std::pair<int, cv::Point3f>(word3Iter->first, point3));
    }

    std::multimap<int, cv::KeyPoint> words;
    const std::vector<cv::KeyPoint> &keypoints = sensorData->keypoints();
    if (wordIds.size() > 0)
    {
        UASSERT(wordIds.size() == keypoints.size());
        unsigned int i = 0;
        for (std::vector<int>::iterator iter = wordIds.begin(); iter != wordIds.end() && i < keypoints.size(); ++iter, ++i)
        {
            words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
        }
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= _minInliers && words.size() >= _minInliers)
    {
        std::vector<int> matches;
        std::vector<int> inliers;
    
        transform = rtabmap::util3d::estimateMotion3DTo2D(
                        uMultimapToMapUnique(words3),
                        uMultimapToMapUnique(words),
                        cameraModel, // TODO: cameraModel.localTransform has to be the same for all images
                        _minInliers,
                        _iterations,
                        _pnpReprojError,
                        _pnpFlags,
                        _pnpRefineIterations,
                        oldSigPose, // use the old signature's pose as a guess
                        std::map<int, cv::Point3f>(),
                        &variance,
                        &matches,
                        &inliers);
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            msg = uFormat("Not enough inliers %d/%d between the old signature %d and the new image", inliersCount, _minInliers, oldSig->getId());
            UINFO(msg.c_str());
        }
    }
    else
    {
        msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)", (int)words3.size(), (int)words.size(), _minInliers);
        UINFO(msg.c_str());
    }

    // TODO check RegistrationVis.cpp to see whether rotation check is necessary

    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    return transform;
}
