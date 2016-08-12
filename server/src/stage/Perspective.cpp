#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <pcl/common/transforms.h>
#include <QCoreApplication>
#include <QDebug>
#include <cassert>
#include "stage/Perspective.h"
#include "event/SignatureEvent.h"
#include "event/LocationEvent.h"
#include "event/FailureEvent.h"
#include "data/PerfData.h"
#include "util/Time.h"

Perspective::Perspective() :
    _vis(nullptr),
    _httpServer(nullptr)
{
}

Perspective::~Perspective()
{
    _vis = nullptr;
    _httpServer = nullptr;
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
        std::unique_ptr<SensorData> sensorData = signatureEvent->takeSensorData();
        std::vector< std::unique_ptr<Signature> > signatures = signatureEvent->takeSignatures();
        std::unique_ptr<PerfData> perfData = signatureEvent->takePerfData();
        const void *session = signatureEvent->getSession();
        std::unique_ptr<Transform> pose(new Transform);
        perfData->perspectiveStart = getTime();
        *pose = localize(*wordIds, *sensorData, *(signatures.at(0)));
        perfData->perspectiveEnd = getTime();
        // a null pose notify that loc could not be computed
        if (pose->isNull() == false)
        {
            QCoreApplication::postEvent(_vis, new LocationEvent(signatures.at(0)->getDbId(), std::move(sensorData), std::move(pose), std::move(perfData), session));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(session));
        }
        return true;
    }
    return QObject::event(event);
}

Transform Perspective::localize(const std::vector<int> &wordIds, const SensorData &sensorData, const Signature &oldSig) const
{
    int minInliers = 3;

    const CameraModel &cameraModel = sensorData.getCameraModel();
    assert(!sensorData.getImage().empty());

    Transform transform;

    int inliersCount = 0;

    std::multimap<int, cv::Point3f> words3;

    const Transform &oldSigPose = oldSig.getPose();

    const std::multimap<int, cv::Point3f> &sigWords3 = oldSig.getWords3();
    std::multimap<int, cv::Point3f>::const_iterator word3Iter;
    for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
    {
        pcl::PointXYZ localPointPCL(word3Iter->second.x, word3Iter->second.y, word3Iter->second.z);
        pcl::PointXYZ globalPointPCL = pcl::transformPoint(localPointPCL, oldSigPose.toEigen3f());
        cv::Point3f globalPointCV = cv::Point3f(globalPointPCL.x, globalPointPCL.y, globalPointPCL.z);
        words3.insert(std::pair<int, cv::Point3f>(word3Iter->first, globalPointCV));
    }

    std::multimap<int, cv::KeyPoint> words;
    const std::vector<cv::KeyPoint> &keypoints = sensorData.keypoints();
    if (wordIds.size() > 0)
    {
        assert(wordIds.size() == keypoints.size());
        unsigned int i = 0;
        for (auto iter = wordIds.begin(); iter != wordIds.end() && i < keypoints.size(); ++iter, ++i)
        {
            words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
        }
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= minInliers && words.size() >= minInliers)
    {
        std::vector<int> inliers;

        transform = estimateMotion3DTo2D(
                        uMultimapToMapUnique(words3),
                        uMultimapToMapUnique(words),
                        cameraModel, // TODO: cameraModel.localTransform has to be the same for all images
                        oldSigPose, // use the old signature's pose as a guess
                        &inliers,
                        minInliers);
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            std::cout << "Not enough inliers " << inliersCount << "/" << minInliers << " between the old signature " << oldSig.getId() << " and the new image" << std::endl;
        }
    }
    else
    {
        std::cout << "Not enough features in images (old=" << words3.size() << ", new=" << words.size() << ", min=" << minInliers << ")" << std::endl;
    }

    // TODO check RegistrationVis.cpp to see whether rotation check is necessary

    qDebug() << "transform= " << transform.prettyPrint().c_str();
    return transform;
}

Transform Perspective::estimateMotion3DTo2D(
    const std::map<int, cv::Point3f> &words3A,
    const std::map<int, cv::KeyPoint> &words2B,
    const CameraModel &cameraModel,
    const Transform &guess,
    std::vector<int> *inliersOut,
    int minInliers) const
{
    assert(cameraModel.isValidForProjection());
    assert(!guess.isNull());
    Transform transform;
    std::vector<int> matches, inliers;

    // find correspondences
    std::vector<int> ids = uKeys(words2B);
    std::vector<cv::Point3f> objectPoints(ids.size());
    std::vector<cv::Point2f> imagePoints(ids.size());
    int oi = 0;
    matches.resize(ids.size());
    for (unsigned int i = 0; i < ids.size(); ++i)
    {
        std::map<int, cv::Point3f>::const_iterator iter = words3A.find(ids[i]);
        if (iter != words3A.end() && std::isfinite(iter->second.x) && std::isfinite(iter->second.y) && std::isfinite(iter->second.z))
        {
            const cv::Point3f &pt = iter->second;
            objectPoints[oi].x = pt.x;
            objectPoints[oi].y = pt.y;
            objectPoints[oi].z = pt.z;
            imagePoints[oi] = words2B.find(ids[i])->second.pt;
            matches[oi++] = ids[i];
        }
    }

    objectPoints.resize(oi);
    imagePoints.resize(oi);
    matches.resize(oi);

    qDebug() << "words3A=" << words3A.size() << " words2B= " << words2B.size() << " matches=" << matches.size();

    if ((int)matches.size() >= minInliers)
    {
        //PnPRansac
        cv::Mat K = cameraModel.K();
        cv::Mat D = cameraModel.D();
        Transform guessCameraFrame = (guess * cameraModel.localTransform()).inverse();
        cv::Mat R = (cv::Mat_<double>(3, 3) <<
                     (double)guessCameraFrame.r11(), (double)guessCameraFrame.r12(), (double)guessCameraFrame.r13(),
                     (double)guessCameraFrame.r21(), (double)guessCameraFrame.r22(), (double)guessCameraFrame.r23(),
                     (double)guessCameraFrame.r31(), (double)guessCameraFrame.r32(), (double)guessCameraFrame.r33());

        cv::Mat rvec(1, 3, CV_64FC1);
        cv::Rodrigues(R, rvec);
        cv::Mat tvec = (cv::Mat_<double>(1, 3) <<
                        (double)guessCameraFrame.x(), (double)guessCameraFrame.y(), (double)guessCameraFrame.z());

        cv::solvePnPRansac(
            objectPoints,
            imagePoints,
            K,
            D,
            rvec,
            tvec,
            true,
            100,
            8.0,
            0.99,
            inliers,
            cv::SOLVEPNP_ITERATIVE); // cv::SOLVEPNP_EPNP
        // TODO check RTABMAp refine model code

        if ((int)inliers.size() >= minInliers)
        {
            cv::Rodrigues(rvec, R);
            Transform pnp(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
                          R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
                          R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2));

            transform = (cameraModel.localTransform() * pnp).inverse();

            // TODO: compute variance (like in PCL computeVariance() method of sac_model.h)
        }
    }

    if (inliersOut)
    {
        inliersOut->resize(inliers.size());
        for (unsigned int i = 0; i < inliers.size(); ++i)
        {
            inliersOut->at(i) = matches[inliers[i]];
        }
    }

    return transform;
}
