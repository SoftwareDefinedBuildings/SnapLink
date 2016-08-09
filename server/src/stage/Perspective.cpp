#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/util3d.h>
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
#include "data/PerfData.h"
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

Transform Perspective::localize(const std::vector<int> &wordIds, const rtabmap::SensorData &sensorData, const Signature &oldSig) const
{
    const rtabmap::CameraModel &cameraModel = sensorData.cameraModels()[0];
    UASSERT(!sensorData.imageRaw().empty());

    Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words3;

    const Transform &oldSigPose = oldSig.getPose();

    const std::multimap<int, cv::Point3f> &sigWords3 = oldSig.getWords3();
    std::multimap<int, cv::Point3f>::const_iterator word3Iter;
    for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
    {
        cv::Point3f point3 = rtabmap::util3d::transformPoint(word3Iter->second, rtabmap::Transform::fromEigen4f(oldSigPose.toEigen4f()));
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

        transform = estimateMotion3DTo2D(
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

Transform Perspective::estimateMotion3DTo2D(
            const std::map<int, cv::Point3f> & words3A,
            const std::map<int, cv::KeyPoint> & words2B,
            const rtabmap::CameraModel & cameraModel,
            int minInliers,
            int iterations,
            double reprojError,
            int flagsPnP,
            int refineIterations,
            const Transform & guess,
            const std::map<int, cv::Point3f> & words3B,
            double * varianceOut,
            std::vector<int> * matchesOut,
            std::vector<int> * inliersOut) const
{
    UASSERT(cameraModel.isValidForProjection());
    UASSERT(!guess.isNull());
    Transform transform;
    std::vector<int> matches, inliers;

    if(varianceOut)
    {
        *varianceOut = 1.0;
    }

    // find correspondences
    std::vector<int> ids = uKeys(words2B);
    std::vector<cv::Point3f> objectPoints(ids.size());
    std::vector<cv::Point2f> imagePoints(ids.size());
    int oi=0;
    matches.resize(ids.size());
    for(unsigned int i=0; i<ids.size(); ++i)
    {
        std::map<int, cv::Point3f>::const_iterator iter=words3A.find(ids[i]);
        if(iter != words3A.end() && rtabmap::util3d::isFinite(iter->second))
        {
            const cv::Point3f & pt = iter->second;
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

    UDEBUG("words3A=%d words2B=%d matches=%d words3B=%d",
            (int)words3A.size(), (int)words2B.size(), (int)matches.size(), (int)words3B.size());

    if((int)matches.size() >= minInliers)
    {
        //PnPRansac
        cv::Mat K = cameraModel.K();
        cv::Mat D = cameraModel.D();
        Transform guessCameraFrame = (guess * Transform::fromEigen4f(cameraModel.localTransform().toEigen4f())).inverse();
        cv::Mat R = (cv::Mat_<double>(3,3) <<
                (double)guessCameraFrame.r11(), (double)guessCameraFrame.r12(), (double)guessCameraFrame.r13(),
                (double)guessCameraFrame.r21(), (double)guessCameraFrame.r22(), (double)guessCameraFrame.r23(),
                (double)guessCameraFrame.r31(), (double)guessCameraFrame.r32(), (double)guessCameraFrame.r33());

        cv::Mat rvec(1,3, CV_64FC1);
        cv::Rodrigues(R, rvec);
        cv::Mat tvec = (cv::Mat_<double>(1,3) <<
                (double)guessCameraFrame.x(), (double)guessCameraFrame.y(), (double)guessCameraFrame.z());

        rtabmap::util3d::solvePnPRansac(
                objectPoints,
                imagePoints,
                K,
                D,
                rvec,
                tvec,
                true,
                iterations,
                reprojError,
                minInliers, // min inliers
                inliers,
                flagsPnP,
                refineIterations);

        if((int)inliers.size() >= minInliers)
        {
            cv::Rodrigues(rvec, R);
            Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
                           R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
                           R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

            transform = (Transform::fromEigen4f(cameraModel.localTransform().toEigen4f()) * pnp).inverse();

            // compute variance (like in PCL computeVariance() method of sac_model.h)
            if(varianceOut && words3B.size())
            {
                std::vector<float> errorSqrdDists(inliers.size());
                oi = 0;
                for(unsigned int i=0; i<inliers.size(); ++i)
                {
                    std::map<int, cv::Point3f>::const_iterator iter = words3B.find(matches[inliers[i]]);
                    if(iter != words3B.end() && rtabmap::util3d::isFinite(iter->second))
                    {
                        const cv::Point3f & objPt = objectPoints[inliers[i]];
                        cv::Point3f newPt = rtabmap::util3d::transformPoint(iter->second, rtabmap::Transform::fromEigen4f(transform.toEigen4f()));
                        errorSqrdDists[oi] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);
                        //ignore very very far features (stereo)
                        if(errorSqrdDists[oi] < 100.0f)
                        {
                            ++oi;
                        }
                    }
                }
                errorSqrdDists.resize(oi);
                if(errorSqrdDists.size())
                {
                    std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
                    double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
                    *varianceOut = 2.1981 * median_error_sqr;
                }
            }
            else if(varianceOut)
            {
                // compute variance, which is the rms of reprojection errors
                std::vector<cv::Point2f> imagePointsReproj;
                cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePointsReproj);
                float err = 0.0f;
                for(unsigned int i=0; i<inliers.size(); ++i)
                {
                    err += uNormSquared(imagePoints.at(inliers[i]).x - imagePointsReproj.at(inliers[i]).x, imagePoints.at(inliers[i]).y - imagePointsReproj.at(inliers[i]).y);
                }
                *varianceOut = std::sqrt(err/float(inliers.size()));
            }
        }
    }

    if(matchesOut)
    {
        *matchesOut = matches;
    }
    if(inliersOut)
    {
        inliersOut->resize(inliers.size());
        for(unsigned int i=0; i<inliers.size(); ++i)
        {
            inliersOut->at(i) = matches[inliers[i]];
        }
    }

    return transform;
}
