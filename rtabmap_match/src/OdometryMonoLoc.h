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

#ifndef ODOMETRY_MONO_LOC_H_
#define ODOMETRY_MONO_LOC_H_

#include <rtabmap/core/RtabmapExp.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class UTimer;

namespace rtabmap {

class Feature2D;
class OdometryInfo;

// inherit from OdometryMono rather than Odometry because RTABMap checks whether the instance of Odometry is OdometryMono
class RTABMAP_EXP OdometryMonoLoc : public OdometryMono
{
public:
    OdometryMonoLoc(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
    virtual ~OdometryMonoLoc();
    virtual void reset(const Transform & initialPose);

private:
    virtual Transform computeTransform(const SensorData & data, OdometryInfo * info = 0);
private:
    //Parameters:
    int flowWinSize_;
    int flowIterations_;
    double flowEps_;
    int flowMaxLevel_;

    Memory * memory_;
    int localHistoryMaxSize_;
    float initMinFlow_;
    float initMinTranslation_;
    float minTranslation_;
    float fundMatrixReprojError_;
    float fundMatrixConfidence_;

    cv::Mat refDepth_;
    std::map<int, cv::Point2f> cornersMap_;
    std::multimap<int, cv::Point3f> localMap_;
    std::map<int, std::multimap<int, pcl::PointXYZ> > keyFrameWords3D_;
    std::map<int, Transform> keyFramePoses_;
    float maxVariance_;
};

} /* namespace rtabmap */
#endif /* ODOMETRY_MONO_LOC_H_ */
