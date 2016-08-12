#include <QCoreApplication>
#include "stage/FeatureExtraction.h"
#include "event/QueryEvent.h"
#include "event/FeatureEvent.h"
#include "util/Time.h"

bool FeatureExtraction::init()
{
    int minHessian = 400;
    _detector = cv::xfeatures2d::SURF::create(minHessian);

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
        std::unique_ptr<SensorData> sensorData = queryEvent->takeSensorData();
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

void FeatureExtraction::extractFeatures(SensorData &sensorData) const
{
    const cv::Mat &image = sensorData.getImage(); // OpenCV uses a shared pointer internally

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    _detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);

    sensorData.setFeatures(keypoints, descriptors);
}
