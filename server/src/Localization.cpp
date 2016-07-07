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

#include "Localization.h"
#include "ImageEvent.h"
#include "LocationEvent.h"
#include "FailureEvent.h"

Localization::Localization() :
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

Localization::~Localization()
{
    _memory = NULL;
    _vis = NULL;
    _httpServer = NULL;
}

bool Localization::init(const rtabmap::ParametersMap &parameters)
{
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisMinInliers(), _minInliers);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisIterations(), _iterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPRefineIterations(), _pnpRefineIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPReprojError(), _pnpReprojError);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPFlags(), _pnpFlags);

    return true;
}

void Localization::setMemory(MemoryLoc *memory)
{
    _memory = memory;
}

void Localization::setVisibility(Visibility *vis)
{
    _vis = vis;
}

void Localization::setHTTPServer(HTTPServer *httpServer)
{
    _httpServer = httpServer;
}

bool Localization::event(QEvent *event)
{
    if (event->type() == ImageEvent::type())
    {
        ImageEvent *imageEvent = static_cast<ImageEvent *>(event);
        rtabmap::Transform pose;
        int dbId;
        bool success = localize(imageEvent->sensorData(), &pose, &dbId, imageEvent->conInfo());
        // a null pose notify that loc could not be computed
        if (success)
        {
            QCoreApplication::postEvent(_vis, new LocationEvent(dbId, imageEvent->sensorData(), pose, imageEvent->conInfo()));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(imageEvent->conInfo()));
        }
        return true;
    }
    return QObject::event(event);
}

bool Localization::localize(rtabmap::SensorData *sensorData, rtabmap::Transform *pose, int *dbId, void *context)
{
    ConnectionInfo *con_info = (ConnectionInfo *) context;

    UASSERT(!sensorData->imageRaw().empty());

    const rtabmap::CameraModel &cameraModel = sensorData->cameraModels()[0];

    // generate kpts
    const rtabmap::Signature *newSig = _memory->createSignature(*sensorData, con_info); // create a new signature, not added to DB
    if (newSig == NULL)
    {
        return false;
    }
    UDEBUG("newWords=%d", (int)newSig->getWords().size());

    con_info->time.search_start = getTime();
    std::vector< std::pair<int, int> > topIds = _memory->findKNearestSignatures(*newSig, TOP_K);
    con_info->time.search += getTime() - con_info->time.search_start; // end of find closest match
    std::pair<int, int> topId = topIds[0];

    // TODO: compare interatively until success
    int topDbId = topId.first;
    int topSigId = topId.second;
    UDEBUG("topDbId: %d, topSigId: %d", topDbId, topSigId);
    con_info->time.pnp_start = getTime();
    *pose = computeGlobalVisualTransform(newSig, topDbId, topSigId);
    con_info->time.pnp += getTime() - con_info->time.pnp_start;
    *dbId = topDbId;

    if (!pose->isNull())
    {
        UDEBUG("global transform = %s", pose->prettyPrint().c_str());
    }
    else
    {
        UWARN("transform is null, using pose of the closest image");
        const rtabmap::Signature *topSig = _memory->getSignature(topDbId, topSigId);
        *pose = topSig->getPose();
    }

    UINFO("output transform = %s using image %d in database %d", pose->prettyPrint().c_str(), topSigId, topDbId);

    delete newSig;

    return !pose->isNull();
}

rtabmap::Transform Localization::computeGlobalVisualTransform(const rtabmap::Signature *newSig, int oldDbId, int oldSigId) const
{
    rtabmap::Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words3;

    const rtabmap::Signature *oldSig = _memory->getSignature(oldDbId, oldSigId);
    const rtabmap::Transform &oldSigPose = oldSig->getPose();

    const std::multimap<int, cv::Point3f> &sigWords3 = oldSig->getWords3();
    std::multimap<int, cv::Point3f>::const_iterator word3Iter;
    for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
    {
        cv::Point3f point3 = rtabmap::util3d::transformPoint(word3Iter->second, oldSigPose);
        words3.insert(std::pair<int, cv::Point3f>(word3Iter->first, point3));
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= _minInliers && (int)newSig->getWords().size() >= _minInliers)
    {
        const rtabmap::CameraModel &cameraModel = newSig->sensorData().cameraModels()[0];

        std::vector<int> matches;
        std::vector<int> inliers;
        transform = rtabmap::util3d::estimateMotion3DTo2D(
                        uMultimapToMapUnique(words3),
                        uMultimapToMapUnique(newSig->getWords()),
                        cameraModel, // TODO: cameraModel.localTransform has to be the same for all images
                        _minInliers,
                        _iterations,
                        _pnpReprojError,
                        _pnpFlags,
                        _pnpRefineIterations,
                        oldSigPose, // use the old signature's pose as a guess
                        uMultimapToMapUnique(newSig->getWords3()),
                        &variance,
                        &matches,
                        &inliers);
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            msg = uFormat("Not enough inliers %d/%d between the old signature %d and %d", inliersCount, _minInliers, oldSig->id(), newSig->id());
            UINFO(msg.c_str());
        }
    }
    else
    {
        msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)", (int)words3.size(), (int)newSig->getWords().size(), _minInliers);
        UINFO(msg.c_str());
    }

    // TODO check RegistrationVis.cpp to see whether rotation check is necessary

    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    return transform;
}
