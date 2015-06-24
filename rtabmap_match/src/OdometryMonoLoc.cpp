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

#include "OdometryMonoLoc.h"
#include "BayesFilter.h"
#include <list>
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Graph.h"

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
    dbPath_(dbPath)
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
    int featureType = Feature2D::kFeatureSurf;
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

    // parameters that makes memory do PnP localization for RGB images
    customParameters.insert(ParametersPair(Parameters::kLccBowPnPEstimation(), "true"));
    customParameters.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false")); 

    memory_ = new Memory(customParameters);
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

    // init bayes filters
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
    Transform output;

    if(data.imageRaw().empty())
    {
        UERROR("Image empty! Cannot compute odometry...");
        return output;
    }

    if(!(((data.cameraModels().size() == 1 && data.cameraModels()[0].isValid()) || data.stereoCameraModel().isValid())))
    {
        UERROR("Odometry cannot be done without calibration or on multi-camera!");
        return output;
    }

    if(data.imageRaw().channels() != 1)
    {
        UERROR("OdometryMonoLoc can only take gray images!");
        return output;
    }

    const CameraModel & cameraModel = data.stereoCameraModel().isValid()?data.stereoCameraModel().left():data.cameraModels()[0];

    UTimer timer;

    int inliers = 0;
    int correspondences = 0;
    int nFeatures = 0;

    Transform mapTransform;

    if(memory_->getWorkingMem().size() >= 1)
    {
        //PnP
        UDEBUG("PnP");

        if(this->isInfoDataFilled() && info)
        {
            info->type = 0;
        }

        // generate kpts
        if(memory_->update(data))
        {
            UDEBUG("");
            const Signature * newS = memory_->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            nFeatures = (int)newS->getWords().size();
            if((int)newS->getWords().size() > this->getMinInliers())
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

                posterior = bayesFilter_->computePosterior(memory_, likelihood);
                if(posterior.size())
                {
                    for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
                    {
                        if(iter->first > 0 && iter->second > highestHypothesis.second)
                        {
                            highestHypothesis = *iter;
                        }
                        //UDEBUG("id = %d, likelihood = %f", iter->first, iter->second);
                    }
                }

                UDEBUG("highestHypothesis.first = %d", highestHypothesis.first);

                // calculate transform between data and the most similar pose
                std::string rejectedMsg;
                int loopClosureVisualInliers = 0;
                double variance = 1;
                SensorData dataFrom = data;
                dataFrom.setId(newS->id());
                SensorData dataTo = memory_->getNodeData(highestHypothesis.first, true);

                if(!dataFrom.depthOrRightRaw().empty() &&
                   !dataTo.depthOrRightRaw().empty() &&
                   dataFrom.id() != Memory::kIdInvalid &&
                   dataTo.id() != Memory::kIdInvalid)
                {
                    // really?  
                }
                else
                {
                    UDEBUG("Calculate map transform");
                    Transform transform = memory_->computeVisualTransform(highestHypothesis.first, newS->id(), &rejectedMsg, &loopClosureVisualInliers, &variance);
                    const Signature * mostSimilarS = memory_->getSignature(highestHypothesis.first);
                    mapTransform = mostSimilarS->getPose() * transform.inverse();// * newS->getPose().inverse(); // this is the final R and t

                }
            }

            // remove new words from dictionary
            memory_->deleteLocation(newS->id());
        }
    }
    else
    {
        UERROR("Memory not initialized. memory_->getWorkingMem().size() = %d", memory_->getWorkingMem().size());
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
