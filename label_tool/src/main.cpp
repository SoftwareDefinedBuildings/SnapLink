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

#include <rtabmap/core/Memory.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <stdio.h>

using namespace rtabmap;

void showUsage()
{
    printf("\nUsage:\n"
            "rtabmap-rgbd_mapping database_file imageId x y\n");
    exit(1);
}

std::map<int, Transform> optimizeGraph(Memory &memory)
{
    if (memory.getLastWorkingSignature())
    {
        // Get all IDs linked to last signature (including those in Long-Term Memory)
        std::map<int, int> ids = memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0);

        UINFO("Optimize poses, ids.size() = %d", ids.size());

        // Get all metric constraints (the graph)
        std::map<int, rtabmap::Transform> poses;
        std::multimap<int, rtabmap::Link> links;
        memory.getMetricConstraints(uKeysSet(ids), poses, links);

        // Optimize the graph
        Optimizer::Type optimizerType = Optimizer::kTypeTORO; // options: kTypeTORO, kTypeG2O, kTypeGTSAM, kTypeCVSBA
        Optimizer *graphOptimizer = Optimizer::create(optimizerType);
        std::map<int, Transform> optimizedPoses = graphOptimizer->optimize(poses.begin()->first, poses, links);
        delete graphOptimizer;

        return optimizedPoses;
    }
}

bool convert(int imageId, int x, int y, Memory &memory, std::map<int, Transform> &poses, pcl::PointXYZ &pWorld)
{
    const SensorData &data = memory.getNodeData(imageId, true);
    const CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;
    //std::cout << data.depthRaw().size() << " " << cm.cx() << " " << cm.cy() << " " << cm.fx() << " " << cm.fy() << " " << cm.localTransform().prettyPrint() << std::endl;
    pcl::PointXYZ pLocal = util3d::projectDepthTo3D(data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
    {
        UWARN("Depth value not valid");
        return false;
    }
    std::map<int, Transform>::const_iterator iter = poses.find(imageId);
    if (iter == poses.end() || iter->second.isNull())
    {
        UWARN("Image pose not found or is Null");
        return false;
    }
    Transform poseWorld = iter->second;
    poseWorld = poseWorld * cm.localTransform();
    pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
    return true;
}


int main(int argc, char * argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);
    //ULogger::setLevel(ULogger::kDebug);

    std::string dbfile;
    int imageId;
    int x;
    int y;
    if(argc != 5)
    {
        showUsage();
    }
    else
    {
        dbfile = std::string(argv[argc-4]);
        imageId = std::stoi(std::string(argv[argc-3]));
        x = std::stoi(std::string(argv[argc-2]));
        y = std::stoi(std::string(argv[argc-1]));
    }

    Memory memory;
    if (!memory.init(dbfile, false))
    {
        UERROR("Error init memory");
        return 1;
    }

    std::map<int, Transform> optimizedPoses = optimizeGraph(memory);
    pcl::PointXYZ pWorld;
    if (convert(imageId, x, y, memory, optimizedPoses, pWorld))
    {
        std::cout << pWorld << std::endl;
    }
    else
    {
        std::cout << "Fail to convert" << std::endl;
    }

    return 0;
}
