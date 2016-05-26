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
    std::vector< std::pair<int, int> > topIds = findKNearestSignatures(*newSig, TOP_K);
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
        *pose = _memory->getOptimizedPose(topDbId, topSigId);
    }

    UINFO("output transform = %s using image %d in database %d", pose->prettyPrint().c_str(), topSigId, topDbId);

    delete newSig;

    return !pose->isNull();
}

std::vector< std::pair<int, int> > Localization::findKNearestSignatures(const rtabmap::Signature &signature, int k)
{
    std::map<std::pair<int, int>, float> similarities; // {(db id, sig id): similarity}
    const std::vector<std::map<int, rtabmap::Signature *> > &signatureMaps = _memory->getSignatureMaps();
    for (int dbId = 0; dbId < signatureMaps.size(); dbId++)
    {
        const std::map<int, rtabmap::Signature *> &signatureMap = signatureMaps.at(dbId);
        for (std::map<int, rtabmap::Signature *>::const_iterator sigIter = signatureMap.begin(); sigIter != signatureMap.end(); sigIter++)
        {
            const rtabmap::Signature *s2 = sigIter->second;
            float sim = signature.compareTo(*s2);
            similarities.insert(std::make_pair(std::make_pair(dbId, sigIter->first), sim));
            UDEBUG("dbId: %d, sigId: %d, similarity: %f", dbId, sigIter->first, sim);
        }
    }

    std::vector< std::pair<int, int> > topIds; // [(db id, sig id)]
    if (similarities.size())
    {
        std::vector< std::pair<std::pair<int, int>, float> > top(k);
        std::partial_sort_copy(similarities.begin(),
                               similarities.end(),
                               top.begin(),
                               top.end(),
                               compareSimilarity);
        for (std::vector< std::pair<std::pair<int, int>, float> >::iterator it = top.begin(); it != top.end(); ++it)
        {
            topIds.push_back(it->first);
            //UDEBUG("dbId: %d, sigId: %d", it->first.first, it->first.second);
        }
    }

    return topIds;
}

// float Localization::computeSimilarity(const rtabmap::Signature &s1, const rtabmap::Signature &s2)
// {
//     float similarity = 0.0f;
//     const std::multimap<int, cv::KeyPoint> &words1 = s1.getWords();
//     const std::multimap<int, cv::KeyPoint> &words2 = s2.getWords();
//     if (words1.size() != 0 && words2.size() != 0)
//     {
//         std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
//         rtabmap::EpipolarGeometry::findPairs(words1, words2, pairs);
//         similarity = float(pairs.size());
//     }
//     return similarity;
// }

bool Localization::compareSimilarity(std::pair<std::pair<int, int>, float> const &l, std::pair<std::pair<int, int>, float> const &r)
{
    return l.second > r.second;
}

rtabmap::Transform Localization::computeGlobalVisualTransform(const rtabmap::Signature *newSig, int oldDbId, int oldSigId) const
{
    rtabmap::Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words3;

    const rtabmap::Signature *oldSig = _memory->getSignature(oldDbId, oldSigId);
    const rtabmap::Transform &oldSigPose = _memory->getOptimizedPose(oldDbId, oldSigId);

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
