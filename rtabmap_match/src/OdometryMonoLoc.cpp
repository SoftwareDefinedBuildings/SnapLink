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
#include "rtabmap/core/VWDictionary.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl/common/centroid.h>

#include "OdometryMonoLoc.h"
#include <list>
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Graph.h"
#include "OdometryInfoErr.h"
#include "MemoryLoc.h"

namespace rtabmap {

OdometryMonoLoc::OdometryMonoLoc(const std::string dbPath, const rtabmap::ParametersMap & parameters) :
    Odometry(parameters),
    flowWinSize_(Parameters::defaultOdomFlowWinSize()),
    flowIterations_(Parameters::defaultOdomFlowIterations()),
    flowEps_(Parameters::defaultOdomFlowEps()),
    flowMaxLevel_(Parameters::defaultOdomFlowMaxLevel()),
    stereoWinSize_(Parameters::defaultStereoWinSize()),
    stereoIterations_(Parameters::defaultStereoIterations()),
    stereoEps_(Parameters::defaultStereoEps()),
    stereoMaxLevel_(Parameters::defaultStereoMaxLevel()),
    stereoMaxSlope_(Parameters::defaultStereoMaxSlope()),
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

    Parameters::parse(parameters, Parameters::kStereoWinSize(), stereoWinSize_);
    Parameters::parse(parameters, Parameters::kStereoIterations(), stereoIterations_);
    Parameters::parse(parameters, Parameters::kStereoEps(), stereoEps_);
    Parameters::parse(parameters, Parameters::kStereoMaxLevel(), stereoMaxLevel_);
    Parameters::parse(parameters, Parameters::kStereoMaxSlope(), stereoMaxSlope_);

    Parameters::parse(parameters, Parameters::kOdomMonoInitMinFlow(), initMinFlow_);
    Parameters::parse(parameters, Parameters::kOdomMonoInitMinTranslation(), initMinTranslation_);
    Parameters::parse(parameters, Parameters::kOdomMonoMinTranslation(), minTranslation_);
    Parameters::parse(parameters, Parameters::kOdomMonoMaxVariance(), maxVariance_);

    Parameters::parse(parameters, Parameters::kVhEpRansacParam1(), fundMatrixReprojError_);
    Parameters::parse(parameters, Parameters::kVhEpRansacParam2(), fundMatrixConfidence_);

    // Setup memory
    memoryParameters_.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
    memoryParameters_.insert(ParametersPair(Parameters::kKpRoiRatios(), this->getRoiRatios()));
    memoryParameters_.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    memoryParameters_.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    memoryParameters_.insert(ParametersPair(Parameters::kMemImageKept(), "true"));
    memoryParameters_.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    memoryParameters_.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
    memoryParameters_.insert(ParametersPair(Parameters::kKpTfIdfLikelihoodUsed(), "false"));
    int nn = Parameters::defaultOdomBowNNType();
    float nndr = Parameters::defaultOdomBowNNDR();
    int featureType = Feature2D::kFeatureSurf;
    int maxFeatures = Parameters::defaultOdomMaxFeatures();
    Parameters::parse(parameters, Parameters::kOdomBowNNType(), nn);
    Parameters::parse(parameters, Parameters::kOdomBowNNDR(), nndr);
    Parameters::parse(parameters, Parameters::kOdomFeatureType(), featureType);
    Parameters::parse(parameters, Parameters::kOdomMaxFeatures(), maxFeatures);
    memoryParameters_.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
    memoryParameters_.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
    memoryParameters_.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));
    memoryParameters_.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(maxFeatures)));

    int subPixWinSize = Parameters::defaultOdomSubPixWinSize();
    int subPixIterations = Parameters::defaultOdomSubPixIterations();
    double subPixEps = Parameters::defaultOdomSubPixEps();
    Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize);
    Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations);
    Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps);
    memoryParameters_.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize)));
    memoryParameters_.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations)));
    memoryParameters_.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps)));

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
            memoryParameters_.insert(*iter);
        }
    }

    // parameters that makes memory do PnP localization for RGB images
    memoryParameters_.insert(ParametersPair(Parameters::kLccBowEstimationType(), "1")); // 1 is PnP
    memoryParameters_.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false")); 
    memoryParameters_.insert(ParametersPair(Parameters::kLccBowMinInliers(), "20")); 

    memory_ = new Memory(memoryParameters_);
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

    transformFile.open("transform.csv");
    transformFile << "filename, old_img_id, x, y, z, roll, pitch, yaw, variance" << std::endl;
}

OdometryMonoLoc::~OdometryMonoLoc()
{
    delete memory_;
    transformFile.close();
}

void OdometryMonoLoc::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    memory_->init("", false, ParametersMap());
    localMap_.clear();
    refDepthOrRight_ = cv::Mat();
    cornersMap_.clear();
    keyFrameWords3D_.clear();
    keyFramePoses_.clear();
}

void OdometryMonoLoc::resetSuperOdom()
{
    Odometry::reset(Transform::getIdentity());
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

    // we don't check whether it is really OdometryInfoErr *
    // only OdometryMonoLocThread calls this through Odometry.process
    OdometryInfoErr * infoErr = (OdometryInfoErr *)info;
    if(infoErr == 0)
    {
        UERROR("info has to be not NULL in OdometryMonoLoc");
        return output;
    }

    const CameraModel & cameraModel = data.stereoCameraModel().isValid()?data.stereoCameraModel().left():data.cameraModels()[0];

    UTimer timer;

    if(memory_->getWorkingMem().size() >= 1)
    {
        //PnP
        UDEBUG("PnP");

        if(this->isInfoDataFilled() && infoErr)
        {
            infoErr->type = 0;
        }

        // generate kpts
        if(memory_->update(data))
        {
            UDEBUG("");
            const Signature * newS = memory_->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            if((int)newS->getWords().size() > this->getMinInliers())
            {
                std::map<int, float> likelihood;
                std::pair<int, float> highestHypothesis(0, 0.0f);

                ULOGGER_INFO("computing likelihood...");
                std::list<int> signaturesToCompare = uKeysList(memory_->getWorkingMem());
                UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
                likelihood = memory_->computeLikelihood(newS, signaturesToCompare);
                
                this->adjustLikelihood(likelihood);

                if(likelihood.size())
                {
                    for(std::map<int, float>::const_reverse_iterator iter = likelihood.rbegin(); iter != likelihood.rend(); ++iter)
                    {
                        if(iter->first > 0 && iter->second > highestHypothesis.second)
                        {
                            highestHypothesis = *iter;
                        }
                        UDEBUG("id = %d, likelihood = %f", iter->first, iter->second);
                    }
                }

                UINFO("highestHypothesis.first = %d", highestHypothesis.first);

                // calculate transform between data and the most similar pose
                ParametersMap customParameters = memoryParameters_; // get BOW LCC parameters
                // override some parameters
                uInsert(customParameters, ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
                uInsert(customParameters, ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
                uInsert(customParameters, ParametersPair(Parameters::kMemBinDataKept(), "false"));
                uInsert(customParameters, ParametersPair(Parameters::kMemSTMSize(), "0"));
                uInsert(customParameters, ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
                uInsert(customParameters, ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
                uInsert(customParameters, ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(VWDictionary::kNNBruteForce))); // bruteforce
                uInsert(customParameters, ParametersPair(Parameters::kKpNndrRatio(), "0.8")); 
                uInsert(customParameters, ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(Feature2D::kFeatureSurf))); // FAST/BRIEF
                uInsert(customParameters, ParametersPair(Parameters::kKpWordsPerImage(), "1500"));
                uInsert(customParameters, ParametersPair(Parameters::kKpBadSignRatio(), "0"));
                uInsert(customParameters, ParametersPair(Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
                uInsert(customParameters, ParametersPair(Parameters::kMemGenerateIds(), "false"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowMinInliers(), "4"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowIterations(), "3000"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowPnPReprojError(), "1.0"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowPnPFlags(), "0")); // interactive
                
                MemoryLoc memoryLoc(customParameters);

                std::string rejectedMsg;
                int visualInliers = 0;
                double variance = 1;
                SensorData dataFrom = data;
                dataFrom.setId(newS->id());
                SensorData dataTo = memory_->getNodeData(highestHypothesis.first, true);

                if(!dataTo.depthOrRightRaw().empty() &&
                   dataFrom.id() != Memory::kIdInvalid &&
                   dataTo.id() != Memory::kIdInvalid)
                {
                    UDEBUG("Calculate map transform with raw data");
                    memoryLoc.update(dataTo);
                    memoryLoc.update(dataFrom);
                    Transform transform = memoryLoc.computeGlobalVisualTransform(dataTo.id(), dataFrom.id(), &rejectedMsg, &visualInliers, &variance);

                    float x, y, z, roll, pitch, yaw;
                    transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                    transformFile << infoErr->fileName << ", " << highestHypothesis.first << ", " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << variance << std::endl; 


                    infoErr->oldImgId = highestHypothesis.first;
                    if(!transform.isNull()) {
                        const Signature * mostSimilarS = memory_->getSignature(highestHypothesis.first);
                        output = mostSimilarS->getPose() * transform.inverse(); // this is the final R and t

                        /*
                        CameraModel cmOld = mostSimilarS->sensorData().cameraModels()[0];
                        UDEBUG("cmOld.fx() = %f, cmOld.fy() = %f, cmOld.cx() = %f, cmOld.cy() = %f, cmOld.localTranform() = %s", cmOld.fx(), cmOld.fy(), cmOld.cx(), cmOld.cy(), cmOld.localTransform().prettyPrint().c_str());
                        CameraModel cmNew = newS->sensorData().cameraModels()[0];
                        UDEBUG("cmNew.fx() = %f, cmNew.fy() = %f, cmNew.cx() = %f, cmNew.cy() = %f, cmNew.localTranform() = %s", cmNew.fx(), cmNew.fy(), cmNew.cx(), cmNew.cy(), cmNew.localTransform().prettyPrint().c_str());
                        */
                        UINFO("mostSimilarS->getPose() = %s", mostSimilarS->getPose().prettyPrint().c_str());
                        UINFO("transform = %s", transform.prettyPrint().c_str());
                        UINFO("transform.inverse() = %s", transform.inverse().prettyPrint().c_str());
                        UDEBUG("newS->getPose() = %s", newS->getPose().prettyPrint().c_str());
                        UDEBUG("newS->getPose().inverse() = %s", newS->getPose().inverse().prettyPrint().c_str());
                    }
                    else
                    {
                        UWARN("transform is null, rejectMsg = %s", rejectedMsg.c_str());
                        if(rejectedMsg.find("Not enough inliers ") == 0)
                        {
                            infoErr->err = 2;
                        }
                        else if(rejectedMsg.find("Not enough features in images (old=") == 0)
                        {
                            infoErr->err = 3;
                        }
                        else if(rejectedMsg.find("Too large rotation detected! (roll=") == 0)
                        {
                            infoErr->err = 4;
                        }
                        else
                        {
                            UERROR("Unkonw error");
                            exit(1);
                        }
                    }
                }
                else
                {
                    UWARN("Data incomplete. dataTo.depthOrRightRaw().empty() = %d, dataFrom.id() = %d, dataTo.id() = %d", 
                            dataTo.depthOrRightRaw().empty(), dataFrom.id(), dataTo.id());
                }
            }
            else
            {
                UWARN("new signature doesn't have enough words. newWords=%d ", (int)newS->getWords().size());
                infoErr->err = 1;
                transformFile << infoErr->fileName << ", , , , , , , , " << std::endl; 
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

    if(this->isInfoDataFilled() && infoErr)
    {
        // TODO is this function returning global transform?
    }

    UINFO("output transform = %s", output.prettyPrint().c_str());

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
