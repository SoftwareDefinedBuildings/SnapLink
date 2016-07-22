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

#include "stage/Perspective.h"
#include "event/SignatureEvent.h"
#include "event/LocationEvent.h"
#include "event/FailureEvent.h"
#include "util/Time.h"

Perspective::Perspective() :
    _vis(nullptr),
    _httpServer(nullptr),
    _minInliers(rtabmap::Parameters::defaultVisMinInliers()),
    _iterations(rtabmap::Parameters::defaultVisIterations()),
    _pnpRefineIterations(rtabmap::Parameters::defaultVisPnPRefineIterations()),
    _pnpReprojError(rtabmap::Parameters::defaultVisPnPReprojError()),
    _pnpFlags(rtabmap::Parameters::defaultVisPnPFlags())
{
}

Perspective::~Perspective()
{
    _vis = nullptr;
    _httpServer = nullptr;
}

bool Perspective::init(const rtabmap::ParametersMap &parameters)
{
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisMinInliers(), _minInliers);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisIterations(), _iterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPRefineIterations(), _pnpRefineIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPReprojError(), _pnpReprojError);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPFlags(), _pnpFlags);

    return true;
}

void Perspective::setVisibility(Visibility *vis)
{
    _vis = vis;
}

void Perspective::setHTTPServer(HTTPServer *httpServer)
{
    _httpServer = httpServer;
}

bool Perspective::event(QEvent *event)
{
    if (event->type() == SignatureEvent::type())
    {
        SignatureEvent *signatureEvent = static_cast<SignatureEvent *>(event);
        std::unique_ptr< std::vector<int> > wordIds = signatureEvent->takeWordIds();
        std::unique_ptr<rtabmap::SensorData> sensorData = signatureEvent->takeSensorData();
        std::unique_ptr< std::vector<Signature *> > signatures = signatureEvent->takeSignatures();
        ConnectionInfo *conInfo = signatureEvent->conInfo();
        std::unique_ptr<rtabmap::Transform> pose(new rtabmap::Transform);
        *pose = localize(*wordIds, *sensorData, *(signatures->at(0)), conInfo);
        // a null pose notify that loc could not be computed
        if (pose->isNull() == false)
        {
            QCoreApplication::postEvent(_vis, new LocationEvent(signatures->at(0)->getDbId(), std::move(sensorData), std::move(pose), conInfo));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(signatureEvent->conInfo()));
        }
        return true;
    }
    return QObject::event(event);
}

rtabmap::Transform Perspective::localize(const std::vector<int> &wordIds, const rtabmap::SensorData &sensorData, const Signature &oldSig, void *context) const
{
    ConnectionInfo *con_info = (ConnectionInfo *) context;

    const rtabmap::CameraModel &cameraModel = sensorData.cameraModels()[0];
    UASSERT(!sensorData.imageRaw().empty());

    rtabmap::Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words3;

    const rtabmap::Transform &oldSigPose = oldSig.getPose();

    const std::multimap<int, cv::Point3f> &sigWords3 = oldSig.getWords3();
    std::multimap<int, cv::Point3f>::const_iterator word3Iter;
    for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
    {
        cv::Point3f point3 = rtabmap::util3d::transformPoint(word3Iter->second, oldSigPose);
        words3.insert(std::pair<int, cv::Point3f>(word3Iter->first, point3));
    }

    std::multimap<int, cv::KeyPoint> words;
    const std::vector<cv::KeyPoint> &keypoints = sensorData.keypoints();
    if (wordIds.size() > 0)
    {
        UASSERT(wordIds.size() == keypoints.size());
        unsigned int i = 0;
        for (auto iter = wordIds.begin(); iter != wordIds.end() && i < keypoints.size(); ++iter, ++i)
        {
            words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
        }
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= _minInliers && words.size() >= _minInliers)
    {
        std::vector<int> matches;
        std::vector<int> inliers;

        con_info->time.pnp_start = getTime();
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
        con_info->time.pnp += getTime() - con_info->time.pnp_start;
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            msg = uFormat("Not enough inliers %d/%d between the old signature %d and the new image", inliersCount, _minInliers, oldSig.getId());
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
