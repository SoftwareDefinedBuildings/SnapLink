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

#include "SignatureSearch.h"
#include "WordEvent.h"
#include "SignatureEvent.h"
#include "FailureEvent.h"
#include "Signature.h"

SignatureSearch::SignatureSearch() :
    _memory(NULL),
    _perspective(NULL)
{
}

SignatureSearch::~SignatureSearch()
{
    _memory = NULL;
    _perspective = NULL;
}

void SignatureSearch::setMemory(MemoryLoc *memory)
{
    _memory = memory;
}

void SignatureSearch::setPerspective(Perspective *perspective)
{
    _perspective = perspective;
}

bool SignatureSearch::event(QEvent *event)
{
    if (event->type() == WordEvent::type())
    {
        WordEvent *wordEvent = static_cast<WordEvent *>(event);
        std::vector<Signature *> signatures = searchSignatures(wordEvent->wordIds(), wordEvent->sensorData(), wordEvent->conInfo());
        QCoreApplication::postEvent(_perspective, new SignatureEvent(wordEvent->wordIds(), wordEvent->sensorData(), signatures, wordEvent->conInfo()));
        return true;
    }
    return QObject::event(event);
}

std::vector<Signature *> SignatureSearch::searchSignatures(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, void *context)
{
    ConnectionInfo *con_info = (ConnectionInfo *) context;

    con_info->time.search_start = getTime();
    std::vector<int> topIds = _memory->findKNearestSignatures(wordIds, TOP_K);
    con_info->time.search += getTime() - con_info->time.search_start; // end of find closest match
    int topSigId = topIds[0];
    Signature *topSig = _memory->getSignatures().at(topSigId);

    UDEBUG("topSigId: %d", topSigId);

    std::vector<Signature *> signatures;
    signatures.push_back(topSig);
    return signatures;
}
