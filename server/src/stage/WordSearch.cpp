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
#include <cassert>
#include "stage/WordSearch.h"
#include "event/FeatureEvent.h"
#include "event/WordEvent.h"
#include "data/Signature.h"
#include "data/PerfData.h"
#include "util/Time.h"

WordSearch::WordSearch() :
    _imageSearch(nullptr)
{
}

WordSearch::~WordSearch()
{
    _imageSearch = nullptr;
}

void WordSearch::putWords(std::unique_ptr<Words> &&words)
{
    _words = std::move(words);
}

void WordSearch::setSignatureSearch(SignatureSearch *imageSearch)
{
    _imageSearch = imageSearch;
}

bool WordSearch::event(QEvent *event)
{
    if (event->type() == FeatureEvent::type())
    {
        FeatureEvent *featureEvent = static_cast<FeatureEvent *>(event);
        std::unique_ptr<rtabmap::SensorData> sensorData = featureEvent->takeSensorData();
        std::unique_ptr<PerfData> perfData = featureEvent->takePerfData();
        const void *session = featureEvent->getSession();
        std::unique_ptr< std::vector<int> > wordIds(new std::vector<int>());
        perfData->wordsStart = getTime();
        *wordIds = searchWords(*sensorData);
        perfData->wordsEnd = getTime();
        // a null pose notify that loc could not be computed
        QCoreApplication::postEvent(_imageSearch, new WordEvent(std::move(wordIds), std::move(sensorData), std::move(perfData), session));
        return true;
    }
    return QObject::event(event);
}

// TODO maybe only pass skeypoints, descriptors, and model
std::vector<int> WordSearch::searchWords(const rtabmap::SensorData &sensorData) const
{
    assert(!sensorData.imageRaw().empty());

    const rtabmap::CameraModel &cameraModel = sensorData.cameraModels()[0];

    std::vector<cv::KeyPoint> keypoints = sensorData.keypoints();
    cv::Mat descriptors = sensorData.descriptors();

    std::vector<int> wordIds;
    if (descriptors.rows)
    {
        wordIds = _words->findNNs(descriptors);
    }

    return wordIds;
}
