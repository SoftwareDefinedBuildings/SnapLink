#include <QCoreApplication>
#include "stage/FeatureExtraction.h"
#include "event/ImageEvent.h"
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
    if (event->type() == ImageEvent::type())
    {
        ImageEvent *imageEvent = static_cast<ImageEvent *>(event);
        std::unique_ptr<rtabmap::SensorData> sensorData = imageEvent->takeSensorData();
        SessionInfo *sessionInfo = imageEvent->sessionInfo();
        extractFeatures(*sensorData, sessionInfo);
        QCoreApplication::postEvent(_wordSearch, new FeatureEvent(std::move(sensorData), sessionInfo));
        return true;
    }
    return QObject::event(event);
}

void FeatureExtraction::extractFeatures(rtabmap::SensorData &sensorData, void *context) const
{
    SessionInfo *sessionInfo = (SessionInfo *) context;
    cv::Mat imageMono;
    if (sensorData.imageRaw().channels() == 3)
    {
        cv::cvtColor(sensorData.imageRaw(), imageMono, CV_BGR2GRAY);
    }
    else
    {
        imageMono = sensorData.imageRaw();
    }

    sessionInfo->timeInfo.keypoints_start = getTime(); // start of generateKeypoints
    std::vector<cv::KeyPoint> keypoints = _feature2D->generateKeypoints(imageMono);
    sessionInfo->timeInfo.keypoints += getTime() - sessionInfo->timeInfo.keypoints_start; // end of generateKeypoints

    sessionInfo->timeInfo.descriptors_start = getTime(); // start of generateDescriptors
    cv::Mat descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
    sessionInfo->timeInfo.descriptors += getTime() - sessionInfo->timeInfo.descriptors_start; // end of SURF extraction

    sensorData.setFeatures(keypoints, descriptors);
}
