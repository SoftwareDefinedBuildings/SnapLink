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
    return output;
}


pcl::PointXYZ OdometryMonoLoc::convert(int imgId, int x, int y, float fx, float fy, float cx, float cy, Transform localT)
{
    // 2D to 3D
    SensorData data = memory_->getNodeData(imgId, true);
    cv::Mat depthImage = data.depthRaw();
    pcl::PointXYZ pLocal = util3d::projectDepthTo3D(depthImage, x, y, cx, cy, fx, fy, false); 

    // 3D to world
    const Signature * sig = memory_->getSignature(imgId);
    Transform pose = sig->getPose() * localT ;
    pcl::PointXYZ pWorld = util3d::transformPoint(pLocal, pose);

    return pWorld;
}

} // namespace rtabmap
