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

#include "stage/SignatureSearch.h"
#include "event/WordEvent.h"
#include "event/SignatureEvent.h"
#include "event/FailureEvent.h"
#include "data/Signature.h"

SignatureSearch::SignatureSearch() :
    _perspective(nullptr)
{
}

SignatureSearch::~SignatureSearch()
{
    _perspective = nullptr;
}

void SignatureSearch::setSignatures(std::unique_ptr<Signatures> &&signatures)
{
    _signatures = std::move(signatures);
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
        std::unique_ptr< std::vector<int> > wordIds = wordEvent->takeWordIds();
        std::unique_ptr<rtabmap::SensorData> sensorData = wordEvent->takeSensorData();
        std::unique_ptr<PerfData> perfData = wordEvent->takePerfData();
        const void *session = wordEvent->getSession();
        std::unique_ptr< std::vector<Signature *> > signatures(new std::vector<Signature *>());
        *signatures = searchSignatures(*wordIds, *perfData);
        QCoreApplication::postEvent(_perspective, new SignatureEvent(std::move(wordIds), std::move(sensorData), std::move(signatures), std::move(perfData)));
        return true;
    }
    return QObject::event(event);
}

std::vector<Signature *> SignatureSearch::searchSignatures(const std::vector<int> &wordIds, PerfData &perfData) const
{
    perfData.search_start = getTime();
    std::vector<int> topIds = _signatures->findKNN(wordIds, TOP_K);
    perfData.search += getTime() - perfData.search_start; // end of find closest match
    int topSigId = topIds[0];
    Signature *topSig = _signatures->getSignatures().at(topSigId);

    UDEBUG("topSigId: %d", topSigId);

    std::vector<Signature *> signatures;
    signatures.push_back(topSig);
    return signatures;
}
