/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl/common/centroid.h>

#include "rtabmap/core/Graph.h"

#include "OdometryMonoLoc.h"
#include "BayesFilter.h"
#include <list>

namespace rtabmap {

OdometryMonoLoc::OdometryMonoLoc(const std::string dbPath, const rtabmap::ParametersMap & parameters) :
    OdometryMono(parameters),
    flowWinSize_(Parameters::defaultOdomFlowWinSize()),
    flowIterations_(Parameters::defaultOdomFlowIterations()),
    flowEps_(Parameters::defaultOdomFlowEps()),
    flowMaxLevel_(Parameters::defaultOdomFlowMaxLevel()),
    localHistoryMaxSize_(Parameters::defaultOdomBowLocalHistorySize()),
    initMinFlow_(Parameters::defaultOdomMonoInitMinFlow()),
    initMinTranslation_(Parameters::defaultOdomMonoInitMinTranslation()),
    minTranslation_(Parameters::defaultOdomMonoMinTranslation()),
    fundMatrixReprojError_(Parameters::defaultVhEpRansacParam1()),
    fundMatrixConfidence_(Parameters::defaultVhEpRansacParam2()),
    maxVariance_(Parameters::defaultOdomMonoMaxVariance()),
    dbPath_(dbPath),
    bayesFilter_(0)
{
    Parameters::parse(parameters, Parameters::kOdomFlowWinSize(), flowWinSize_);
    Parameters::parse(parameters, Parameters::kOdomFlowIterations(), flowIterations_);
    Parameters::parse(parameters, Parameters::kOdomFlowEps(), flowEps_);
    Parameters::parse(parameters, Parameters::kOdomFlowMaxLevel(), flowMaxLevel_);
    Parameters::parse(parameters, Parameters::kOdomBowLocalHistorySize(), localHistoryMaxSize_);

    Parameters::parse(parameters, Parameters::kOdomMonoInitMinFlow(), initMinFlow_);
    Parameters::parse(parameters, Parameters::kOdomMonoInitMinTranslation(), initMinTranslation_);
    Parameters::parse(parameters, Parameters::kOdomMonoMinTranslation(), minTranslation_);
    Parameters::parse(parameters, Parameters::kOdomMonoMaxVariance(), maxVariance_);

    Parameters::parse(parameters, Parameters::kVhEpRansacParam1(), fundMatrixReprojError_);
    Parameters::parse(parameters, Parameters::kVhEpRansacParam2(), fundMatrixConfidence_);

    // Setup memory
    ParametersMap customParameters;
    customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
    customParameters.insert(ParametersPair(Parameters::kKpRoiRatios(), this->getRoiRatios()));
    customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    customParameters.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "true"));
    customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    customParameters.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
    customParameters.insert(ParametersPair(Parameters::kKpTfIdfLikelihoodUsed(), "false"));
    int nn = Parameters::defaultOdomBowNNType();
    float nndr = Parameters::defaultOdomBowNNDR();
    int featureType = Parameters::defaultOdomFeatureType();
    int maxFeatures = Parameters::defaultOdomMaxFeatures();
    Parameters::parse(parameters, Parameters::kOdomBowNNType(), nn);
    Parameters::parse(parameters, Parameters::kOdomBowNNDR(), nndr);
    Parameters::parse(parameters, Parameters::kOdomFeatureType(), featureType);
    Parameters::parse(parameters, Parameters::kOdomMaxFeatures(), maxFeatures);
    customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
    customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
    customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));
    customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(maxFeatures)));

    int subPixWinSize = Parameters::defaultOdomSubPixWinSize();
    int subPixIterations = Parameters::defaultOdomSubPixIterations();
    double subPixEps = Parameters::defaultOdomSubPixEps();
    Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize);
    Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations);
    Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps);
    customParameters.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize)));
    customParameters.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations)));
    customParameters.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps)));

    // add only feature stuff
    for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
    {
        std::string group = uSplit(iter->first, '/').front();
        if(group.compare("SURF") == 0 ||
            group.compare("SIFT") == 0 ||
            group.compare("BRIEF") == 0 ||
            group.compare("FAST") == 0 ||
            group.compare("ORB") == 0 ||
            group.compare("FREAK") == 0 ||
            group.compare("GFTT") == 0 ||
            group.compare("BRISK") == 0)
        {
            customParameters.insert(*iter);
        }
    }

    memory_ = new Memory(customParameters);

    customParameters.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
    if(!memory_->init(dbPath_, false, ParametersMap()))
    {
        UERROR("Error initializing the memory for Mono Odometry.");
    }
    else
    {
        // get the graph
        std::map<int, int> ids = memory_->getNeighborsId(memory_->getLastSignatureId(), 0, 0);
        std::map<int, Transform> poses;
        std::multimap<int, Link> links;
        memory_->getMetricConstraints(uKeysSet(ids), poses, links);
    
        //optimize the graph
        graph::TOROOptimizer optimizer;
        std::map<int, Transform> optimizedPoses = optimizer.optimize(poses.begin()->first, poses, links);
    
        // fill the local map
        for(std::map<int, Transform>::iterator posesIter=optimizedPoses.begin();
            posesIter!=optimizedPoses.end();
            ++posesIter)
        {
            const Signature * s = memory_->getSignature(posesIter->first);
            if(s)
            {
                // Transform 3D points accordingly to pose and add them to local map
                const std::multimap<int, pcl::PointXYZ> & words3D = s->getWords3();
                for(std::multimap<int, pcl::PointXYZ>::const_iterator pointsIter=words3D.begin();
                    pointsIter!=words3D.end();
                    ++pointsIter)
                {
                    if(!uContains(localMap_, pointsIter->first))
                    {
                        pcl::PointXYZ pointPCL = util3d::transformPoint(pointsIter->second, posesIter->second);
                        cv::Point3f pointCV(pointPCL.x, pointPCL.y, pointPCL.z);
                        localMap_.insert(std::make_pair(pointsIter->first, pointCV));
                    }
                }
            }
        }
    }

    if(!bayesFilter_)
    {
        bayesFilter_ = new BayesFilter(parameters);
    }
    else
    {
        bayesFilter_->parseParameters(parameters);
    }
}

OdometryMonoLoc::~OdometryMonoLoc()
{
    delete memory_;
    delete bayesFilter_;
}

void OdometryMonoLoc::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    memory_->init("", false, ParametersMap());
    localMap_.clear();
    refDepth_ = cv::Mat();
    cornersMap_.clear();
    keyFrameWords3D_.clear();
    keyFramePoses_.clear();
}

Transform OdometryMonoLoc::computeTransform(const SensorData & data, OdometryInfo * info)
{
    UASSERT(!data.image().empty());
    UASSERT(data.fx());

    UTimer timer;
    Transform output;

    int inliers = 0;
    int correspondences = 0;
    int nFeatures = 0;

    cv::Mat newFrame;
    // convert to grayscale
    if(data.image().channels() > 1)
    {
        cv::cvtColor(data.image(), newFrame, cv::COLOR_BGR2GRAY);
    }
    else
    {
        newFrame = data.image().clone();
    }

    if(memory_->getWorkingMem().size())
    {
        //PnP
        UDEBUG("PnP");

        if(this->isInfoDataFilled() && info)
        {
            info->type = 0;
        }

        // generate kpts
        if(memory_->update(SensorData(newFrame)))
        {
            UDEBUG("");
            bool newPtsAdded = false;
            const Signature * newS = memory_->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            nFeatures = (int)newS->getWords().size();
            UDEBUG("(int)newS->getWords().size() = %d", (int)newS->getWords().size());
            //if((int)newS->getWords().size() > this->getMinInliers())
            {
                std::map<int, float> rawLikelihood;
                std::map<int, float> adjustedLikelihood;
                std::map<int, float> likelihood;
                std::map<int, float> posterior;
                std::pair<int, float> highestHypothesis(0, 0.0f);

                ULOGGER_INFO("computing likelihood...");
                std::list<int> signaturesToCompare = uKeysList(memory_->getWorkingMem());
                UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
                rawLikelihood = memory_->computeLikelihood(newS, signaturesToCompare);

                likelihood = rawLikelihood;
                this->adjustLikelihood(likelihood);
                    
                for(std::map<int, float>::const_reverse_iterator iter = likelihood.rbegin(); iter != likelihood.rend(); ++iter)
                {
                    if(iter->first > 0 && iter->second > highestHypothesis.second)
                    {
                        highestHypothesis = *iter;
                    }
                    UDEBUG("lh.first = %d", iter->first);
                    UDEBUG("lh.second = %d", iter->second);
                }
/*
                posterior = bayesFilter_->computePosterior(memory_, likelihood);
                if(posterior.size())
                {
                    for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
                    {
                        if(iter->first > 0 && iter->second > highestHypothesis.second)
                        {
                            highestHypothesis = *iter;
                        }
                    }
                }
*/
                UDEBUG("highestHypothesis.first = %d", highestHypothesis.first);
                // With the virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
                highestHypothesis.second = 1 - posterior.begin()->second;



                /*
                cv::Mat K = (cv::Mat_<double>(3,3) <<
                    data.fx(), 0, data.cx(),
                    0, data.fy()==0?data.fx():data.fy(), data.cy(),
                    0, 0, 1);
                Transform guess = (this->getPose() * data.localTransform()).inverse();
                cv::Mat R = (cv::Mat_<double>(3,3) <<
                        (double)guess.r11(), (double)guess.r12(), (double)guess.r13(),
                        (double)guess.r21(), (double)guess.r22(), (double)guess.r23(),
                        (double)guess.r31(), (double)guess.r32(), (double)guess.r33());
                cv::Mat rvec(1,3, CV_64FC1);
                cv::Rodrigues(R, rvec);
                cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guess.x(), (double)guess.y(), (double)guess.z());

                std::vector<cv::Point3f> objectPoints;
                std::vector<cv::Point2f> imagePoints;
                std::vector<int> matches;

                UDEBUG("compute PnP from optical flow");

                std::vector<int> ids = uKeys(localMap_);
                objectPoints = uValues(localMap_);

                // compute last projection
                UDEBUG("project points to previous image");
                std::vector<cv::Point2f> prevImagePoints;
                const Signature * prevS = memory_->getSignature(*(++memory_->getStMem().rbegin()));
                Transform prevGuess = (keyFramePoses_.at(prevS->id()) * data.localTransform()).inverse();
                cv::Mat prevR = (cv::Mat_<double>(3,3) <<
                        (double)prevGuess.r11(), (double)prevGuess.r12(), (double)prevGuess.r13(),
                        (double)prevGuess.r21(), (double)prevGuess.r22(), (double)prevGuess.r23(),
                        (double)prevGuess.r31(), (double)prevGuess.r32(), (double)prevGuess.r33());
                cv::Mat prevRvec(1,3, CV_64FC1);
                cv::Rodrigues(prevR, prevRvec);
                cv::Mat prevTvec = (cv::Mat_<double>(1,3) << (double)prevGuess.x(), (double)prevGuess.y(), (double)prevGuess.z());
                cv::projectPoints(objectPoints, prevRvec, prevTvec, K, cv::Mat(), prevImagePoints);

                // compute current projection
                UDEBUG("project points to previous image");
                cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePoints);

                //filter points not in the image and set guess from unique correspondences
                std::vector<cv::Point3f> objectPointsTmp(objectPoints.size());
                std::vector<cv::Point2f> refCorners(objectPoints.size());
                std::vector<cv::Point2f> newCorners(objectPoints.size());
                matches.resize(objectPoints.size());
                int oi=0;
                for(unsigned int i=0; i<objectPoints.size(); ++i)
                {
                    if(uIsInBounds(int(imagePoints[i].x), 0, newFrame.cols) &&
                       uIsInBounds(int(imagePoints[i].y), 0, newFrame.rows) &&
                       uIsInBounds(int(prevImagePoints[i].x), 0, prevS->getImageRaw().cols) &&
                       uIsInBounds(int(prevImagePoints[i].y), 0, prevS->getImageRaw().rows))
                    {
                        refCorners[oi] = prevImagePoints[i];
                        newCorners[oi] = imagePoints[i];
                        if(localMap_.count(ids[i]) == 1)
                        {
                            if(prevS->getWords().count(ids[i]) == 1)
                            {
                                // set guess if unique
                                refCorners[oi] = prevS->getWords().find(ids[i])->second.pt;
                            }
                            if(newS->getWords().count(ids[i]) == 1)
                            {
                                // set guess if unique
                                newCorners[oi] = newS->getWords().find(ids[i])->second.pt;
                            }
                        }
                        objectPointsTmp[oi] = objectPoints[i];
                        matches[oi] = ids[i];
                        ++oi;
                    }
                }
                objectPointsTmp.resize(oi);
                refCorners.resize(oi);
                newCorners.resize(oi);
                matches.resize(oi);

                // Refine imagePoints using optical flow
                std::vector<unsigned char> statusFlowInliers;
                std::vector<float> err;
                UDEBUG("cv::calcOpticalFlowPyrLK() begin");
                cv::calcOpticalFlowPyrLK(
                        prevS->getImageRaw(),
                        newFrame,
                        refCorners,
                        newCorners,
                        statusFlowInliers,
                        err,
                        cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
                        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
                        cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
                UDEBUG("cv::calcOpticalFlowPyrLK() end");

                objectPoints.resize(statusFlowInliers.size());
                imagePoints.resize(statusFlowInliers.size());
                std::vector<int> matchesTmp(statusFlowInliers.size());
                oi = 0;
                for(unsigned int i=0; i<statusFlowInliers.size(); ++i)
                {
                    if(statusFlowInliers[i])
                    {
                        objectPoints[oi] = objectPointsTmp[i];
                        imagePoints[oi] = newCorners[i];
                        matchesTmp[oi] = matches[i];
                        ++oi;

                        if(this->isInfoDataFilled() && info)
                        {
                            cv::KeyPoint kpt;
                            if(newS->getWords().count(matches[i]) == 1)
                            {
                                kpt = newS->getWords().find(matches[i])->second;
                            }
                            kpt.pt = newCorners[i];
                            info->words.insert(std::make_pair(matches[i], kpt));
                        }
                    }
                }
                UDEBUG("Flow inliers= %d/%d", oi, (int)statusFlowInliers.size());
                objectPoints.resize(oi);
                imagePoints.resize(oi);
                matchesTmp.resize(oi);
                matches = matchesTmp;

                if(this->isInfoDataFilled() && info)
                {
                    info->wordMatches.insert(info->wordMatches.end(), matches.begin(), matches.end());
                }
                correspondences = (int)matches.size();

                if((int)matches.size() < this->getMinInliers())
                {
                    UWARN("not enough matches (%d < %d)...", (int)matches.size(), this->getMinInliers());
                }
                else
                {
                    //PnPRansac
                    std::vector<int> inliersV;
                    cv::solvePnPRansac(
                            objectPoints,
                            imagePoints,
                            K,
                            cv::Mat(),
                            rvec,
                            tvec,
                            true,
                            this->getIterations(),
                            this->getPnPReprojError(),
                            0,
                            inliersV,
                            this->getPnPFlags());

                    UDEBUG("inliers=%d/%d", (int)inliersV.size(), (int)objectPoints.size());

                    inliers = (int)inliersV.size();
                    if((int)inliersV.size() < this->getMinInliers())
                    {
                        UWARN("PnP not enough inliers (%d < %d), rejecting the transform...", (int)inliersV.size(), this->getMinInliers());
                    }
                    else
                    {
                        cv::Mat R(3,3,CV_64FC1);
                        cv::Rodrigues(rvec, R);
                        Transform pnp = Transform(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
                                                  R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
                                                  R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));
                        output = this->getPose().inverse() * pnp.inverse() * data.localTransform().inverse();

                        if(this->isInfoDataFilled() && info && inliersV.size())
                        {
                            info->wordInliers.resize(inliersV.size());
                            for(unsigned int i=0; i<inliersV.size(); ++i)
                            {
                                info->wordInliers[i] = matches[inliersV[i]]; // index and ID should match (index starts at 0, ID starts at 1)
                            }
                        }

                        //Find the frame with the most similar features
                        std::set<int> stMem =  memory_->getStMem();
                        stMem.erase(newS->id());
                        std::map<int, float> likelihood = memory_->computeLikelihood(newS, std::list<int>(stMem.begin(), stMem.end()));
                        int maxLikelihoodId = -1;
                        float maxLikelihood = 0;
                        for(std::map<int, float>::iterator iter=likelihood.begin(); iter!=likelihood.end(); ++iter)
                        {
                            if(iter->second > maxLikelihood)
                            {
                                maxLikelihood = iter->second;
                                maxLikelihoodId = iter->first;
                            }
                        }
                        UASSERT(maxLikelihoodId != -1);

                        // Add new points to local map
                        const Signature* previousS = memory_->getSignature(maxLikelihoodId);
                        UASSERT(previousS!=0);
                        Transform cameraTransform = keyFramePoses_.at(previousS->id()).inverse()*this->getPose()*output;
                        UDEBUG("cameraTransform guess=  %s (norm^2=%f)", cameraTransform.prettyPrint().c_str(), cameraTransform.getNormSquared());
                        if(cameraTransform.getNorm() < minTranslation_)
                        {
                            UWARN("Translation with the nearest frame is too small (%f<%f) to add new points to local map",
                                    cameraTransform.getNorm(), minTranslation_);
                        }
                        else
                        {

                            double variance = 0;
                            const std::multimap<int, pcl::PointXYZ> & previousGuess = keyFrameWords3D_.find(previousS->id())->second;
                            std::multimap<int, pcl::PointXYZ> inliers3D = util3d::generateWords3DMono(
                                    previousS->getWords(),
                                    newS->getWords(),
                                    data.fx(), data.fy()?data.fy():data.fx(),
                                    data.cx(), data.cy(),
                                    data.localTransform(),
                                    cameraTransform,
                                    this->getIterations(),
                                    this->getPnPReprojError(),
                                    this->getPnPFlags(),
                                    fundMatrixReprojError_,
                                    fundMatrixConfidence_,
                                    previousGuess,
                                    &variance);

                            if((int)inliers3D.size() < this->getMinInliers())
                            {
                                UWARN("Epipolar geometry not enough inliers (%d < %d), rejecting the transform (%s)...",
                                (int)inliers3D.size(), this->getMinInliers(), cameraTransform.prettyPrint().c_str());
                            }
                            else if(variance == 0 || variance > maxVariance_)
                            {
                                UWARN("Variance too high %f (max = %f)", variance, maxVariance_);
                            }
                            else
                            {
                                UDEBUG("inliers3D=%d/%d variance=  %f", inliers3D.size(), newS->getWords().size(), variance);
                                Transform newPose = keyFramePoses_.at(previousS->id())*cameraTransform;
                                UDEBUG("cameraTransform=  %s", cameraTransform.prettyPrint().c_str());

                                std::multimap<int, cv::Point3f> wordsToAdd;
                                for(std::multimap<int, pcl::PointXYZ>::iterator iter=inliers3D.begin();
                                    iter != inliers3D.end();
                                    ++iter)
                                {
                                    // transform inliers3D in new signature referential
                                    iter->second = util3d::transformPoint(iter->second, cameraTransform.inverse());

                                    if(!uContains(localMap_, iter->first))
                                    {
                                        //UDEBUG("Add new point %d to local map", iter->first);
                                        pcl::PointXYZ newPt = util3d::transformPoint(iter->second, newPose);
                                        wordsToAdd.insert(std::make_pair(iter->first, cv::Point3f(newPt.x, newPt.y, newPt.z)));
                                    }
                                }

                                if((int)wordsToAdd.size())
                                {
                                    localMap_.insert(wordsToAdd.begin(), wordsToAdd.end());
                                    newPtsAdded = true;
                                    UDEBUG("Added %d words", (int)wordsToAdd.size());
                                }

                                if(newPtsAdded)
                                {
                                    keyFrameWords3D_.insert(std::make_pair(newS->id(), inliers3D));
                                    keyFramePoses_.insert(std::make_pair(newS->id(), newPose));

                                    // keep only the two last signatures
                                    while(localHistoryMaxSize_ && (int)localMap_.size() > localHistoryMaxSize_ && memory_->getStMem().size()>2)
                                    {
                                        int nodeId = *memory_->getStMem().begin();
                                        std::list<int> removedPts;
                                        memory_->deleteLocation(nodeId, &removedPts);
                                        keyFrameWords3D_.erase(nodeId);
                                        keyFramePoses_.erase(nodeId);
                                        for(std::list<int>::iterator iter = removedPts.begin(); iter!=removedPts.end(); ++iter)
                                        {
                                            localMap_.erase(*iter);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }*/
            }

            if(!newPtsAdded)
            {
                // remove new words from dictionary
                memory_->deleteLocation(newS->id());
            }
        }
    }
    else
    {
        UERROR("Memory not initialized");
    }

    memory_->emptyTrash();

    if(this->isInfoDataFilled() && info)
    {
        //info->variance = variance;
        info->inliers = inliers;
        info->matches = correspondences;
        info->features = nFeatures;
        info->localMapSize = (int)localMap_.size();
        info->localMap = localMap_;
    }

    UINFO("Odom update=%fs tf=[%s] inliers=%d/%d, local_map[%d]=%d, accepted=%s",
            timer.elapsed(),
            output.prettyPrint().c_str(),
            inliers,
            correspondences,
            (int)memory_->getStMem().size(),
            (int)localMap_.size(),
            !output.isNull()?"true":"false");

    return output;
}

void OdometryMonoLoc::adjustLikelihood(std::map<int, float> & likelihood) const
{
    ULOGGER_DEBUG("likelihood.size()=%d", likelihood.size());
    UTimer timer;
    timer.start();
    if(likelihood.size()==0)
    {
        return;
    }

    // Use only non-null values (ignore virtual place)
    std::list<float> values;
    bool likelihoodNullValuesIgnored = true;
    for(std::map<int, float>::iterator iter = ++likelihood.begin(); iter!=likelihood.end(); ++iter)
    {
        if((iter->second >= 0 && !likelihoodNullValuesIgnored) ||
           (iter->second > 0 && likelihoodNullValuesIgnored))
        {
            values.push_back(iter->second);
        }
    }
    UDEBUG("values.size=%d", values.size());

    float mean = uMean(values);
    float stdDev = std::sqrt(uVariance(values, mean));


    //Adjust likelihood with mean and standard deviation (see Angeli phd)
    float epsilon = 0.0001;
    float max = 0.0f;
    int maxId = 0;
    for(std::map<int, float>::iterator iter=++likelihood.begin(); iter!= likelihood.end(); ++iter)
    {
        float value = iter->second;
        if(value > mean+stdDev && mean)
        {
            iter->second = (value-(stdDev-epsilon))/mean;
            if(value > max)
            {
                max = value;
                maxId = iter->first;
            }
        }
        else if(value == 1.0f && stdDev == 0)
        {
            iter->second = 1.0f;
            if(value > max)
            {
                max = value;
                maxId = iter->first;
            }
        }
        else
        {
            iter->second = 1.0f;
        }
    }

    if(stdDev > epsilon && max)
    {
        likelihood.begin()->second = mean/stdDev + 1.0f;
    }
    else
    {
        likelihood.begin()->second = 2.0f; //2 * std dev
    }

    double time = timer.ticks();
    UDEBUG("mean=%f, stdDev=%f, max=%f, maxId=%d, time=%fs", mean, stdDev, max, maxId, time);
}

} // namespace rtabmap
