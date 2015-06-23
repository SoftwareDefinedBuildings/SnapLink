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

#ifndef UTIL3D_FEATURES_ASYM_H_
#define UTIL3D_FEATURES_ASYM_H_

#include <rtabmap/core/RtabmapExp.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/Transform.h>
#include <list>
#include <map>

namespace rtabmap
{

namespace util3d
{
std::multimap<int, pcl::PointXYZ> RTABMAP_EXP generateWords3DMonoAsym(
        const std::multimap<int, cv::KeyPoint> & refWords,
        const std::multimap<int, cv::KeyPoint> & nextWords,
        float ref_fx,
        float ref_fy,
        float ref_cx,
        float ref_cy,
        const Transform & ref_localTransform,
        float next_fx,
        float next_fy,
        float next_cx,
        float next_cy,
        const Transform & next_localTransform,
        Transform & cameraTransform,
        int pnpIterations = 100,
        float pnpReprojError = 8.0f,
        int pnpFlags = 0, // cv::SOLVEPNP_ITERATIVE
        float ransacParam1 = 3.0f,
        float ransacParam2 = 0.99f,
        const std::multimap<int, pcl::PointXYZ> & refGuess3D = std::multimap<int, pcl::PointXYZ>(),
        double * variance = 0);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FEATURES_ASYM_H_ */
