#include <QCoreApplication>
#include "stage/FeatureExtraction.h"
#include "event/QueryEvent.h"
#include "event/FeatureEvent.h"
#include "util/Time.h"

FeatureExtraction::FeatureExtraction() :
    _feature2D(nullptr)
{
}

FeatureExtraction::~FeatureExtraction()
{
    delete _feature2D;
    _feature2D = nullptr;
}

bool FeatureExtraction::init(const rtabmap::ParametersMap &parameters)
{
    _feature2D = rtabmap::Feature2D::create(parameters);

    return true;
}

void FeatureExtraction::setWordSearch(WordSearch *wordSearch)
{
    _wordSearch = wordSearch;
}

bool FeatureExtraction::event(QEvent *event)
{
    if (event->type() == QueryEvent::type())
    {
        QueryEvent *queryEvent = static_cast<QueryEvent *>(event);
        std::unique_ptr<rtabmap::SensorData> sensorData = queryEvent->takeSensorData();
        std::unique_ptr<PerfData> perfData = queryEvent->takePerfData();
        const void *session = queryEvent->getSession();

        perfData->featuresStart = getTime();
        extractFeatures(*sensorData);
        perfData->featuresEnd = getTime();

        QCoreApplication::postEvent(_wordSearch, new FeatureEvent(std::move(sensorData), std::move(perfData), session));

        return true;
    }
    return QObject::event(event);
}

void FeatureExtraction::extractFeatures(rtabmap::SensorData &sensorData) const
{
    cv::Mat image = sensorData.imageRaw(); // OpenCV uses a shared pointer internally

    std::vector<cv::KeyPoint> keypoints = _feature2D->generateKeypoints(image);
    cv::Mat descriptors = _feature2D->generateDescriptors(image, keypoints);

    sensorData.setFeatures(keypoints, descriptors);
}
