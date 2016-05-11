#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Rtabmap.h>
#include <QCoreApplication>

#include "Localization.h"
#include "ImageEvent.h"
#include "LocationEvent.h"
#include "FailureEvent.h"

Localization::Localization() :
    _memories(NULL),
    _vis(NULL),
    _httpServer(NULL)
{
}

Localization::~Localization()
{
    _memories->clear();
    _memories = NULL;
    _vis = NULL;
    _httpServer = NULL;
}

void Localization::setMemories(std::vector<MemoryLoc *> *memories)
{
    _memories = memories;
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

    std::map<std::pair<int, int>, float> similarities; // {{db id: sig id}: similarity}

    for (int i = 0; i < _memories->size(); ++i)
    {
        MemoryLoc *memory = _memories->at(i);
        // generate kpts
        if (memory->update(*sensorData, con_info))
        {
            con_info->time.search_start = getTime();
            UDEBUG("");
            const rtabmap::Signature *newS = memory->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            std::list<int> signaturesToCompare = uKeysList(memory->getSignatures());
            signaturesToCompare.remove(newS->id());
            UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
            std::map<int, float> similarity = memory->computeSimilarity(newS, signaturesToCompare);
            similarity.erase(-1);
            for (std::map<int, float>::const_iterator j = similarity.begin(); j != similarity.end(); ++j)
            {
                similarities.insert(std::make_pair(std::make_pair(i, j->first), j->second));
            }
            con_info->time.search += getTime() - con_info->time.search_start; // end of find closest match
        }
    }

    std::vector< std::pair<int, int> > topIds;
    std::pair<int, int> topId;
    if (similarities.size())
    {
        std::vector< std::pair<std::pair<int, int>, float> > top(TOP_K);
        std::partial_sort_copy(similarities.begin(),
                               similarities.end(),
                               top.begin(),
                               top.end(),
                               compareLikelihood);
        for (std::vector< std::pair<std::pair<int, int>, float> >::iterator it = top.begin(); it != top.end(); ++it)
        {
            topIds.push_back(it->first);
        }
        topId = topIds[0];
        UDEBUG("topId: %d", topId);
    }

    // TODO: compare interatively until success
    int topDbId = topId.first;
    int topSigId = topId.second;
    const rtabmap::Signature *newS = _memories->at(topDbId)->getLastWorkingSignature();
    con_info->time.pnp_start = getTime();
    *pose = _memories->at(topDbId)->computeGlobalVisualTransform(topSigId, newS->id());
    con_info->time.pnp += getTime() - con_info->time.pnp_start;
    *dbId = topDbId;

    if (!pose->isNull())
    {
        UDEBUG("global transform = %s", pose->prettyPrint().c_str());
    }
    else
    {
        UWARN("transform is null, using pose of the closest image");
        //*pose = _memories->at(topDbId)->getOptimizedPose(topSigId);
    }

    // remove new words from dictionary
    for (int i = 0; i < _memories->size(); ++i)
    {
        MemoryLoc *memory = _memories->at(i);
        const rtabmap::Signature *newS = memory->getLastWorkingSignature();
        memory->deleteLocation(newS->id());
        memory->emptyTrash();
    }

    UINFO("output transform = %s using image %d in database %d", pose->prettyPrint().c_str(), topSigId, topDbId);

    return !pose->isNull();
}

bool Localization::compareLikelihood(std::pair<std::pair<int, int>, float> const &l, std::pair<std::pair<int, int>, float> const &r)
{
    return l.second > r.second;
}
