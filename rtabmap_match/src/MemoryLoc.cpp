#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>
#include <pcl/io/ply_io.h>
#include <algorithm>
#include <iostream>
#include <fstream>

#include "dbscan.h"
#include "Utility.h"
#include "MemoryLoc.h"

namespace rtabmap {

MemoryLoc::MemoryLoc(const ParametersMap & parameters) :
    Memory(parameters),
    _bowMinInliers(Parameters::defaultLccBowMinInliers()),
    _bowIterations(Parameters::defaultLccBowIterations()),
    _bowRefineIterations(Parameters::defaultLccBowRefineIterations()),
    _bowPnPReprojError(Parameters::defaultLccBowPnPReprojError()),
    _bowPnPFlags(Parameters::defaultLccBowPnPFlags())
{
    Parameters::parse(parameters, Parameters::kLccBowMinInliers(), _bowMinInliers);
    Parameters::parse(parameters, Parameters::kLccBowIterations(), _bowIterations);
    Parameters::parse(parameters, Parameters::kLccBowRefineIterations(), _bowRefineIterations);
    Parameters::parse(parameters, Parameters::kLccBowPnPReprojError(), _bowPnPReprojError);
    Parameters::parse(parameters, Parameters::kLccBowPnPFlags(), _bowPnPFlags);

    UASSERT_MSG(_bowMinInliers >= 1, uFormat("value=%d", _bowMinInliers).c_str());
    UASSERT_MSG(_bowIterations > 0, uFormat("value=%d", _bowIterations).c_str());
    
    _voxelSize = 0.03f;
    _filteringRadius = 0.1f;
    _filteringMinNeighbors = 5;
    _MLSRadius = 0.1f;
    _MLSpolygonialOrder = 2;
    _MLSUpsamplingMethod = 0; // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
    _MLSUpsamplingRadius = 0.0f;   // SAMPLE_LOCAL_PLANE
    _MLSUpsamplingStep = 0.0f;     // SAMPLE_LOCAL_PLANE
    _MLSPointDensity = 0;            // RANDOM_UNIFORM_DENSITY
    _MLSDilationVoxelSize = 0.04f;  // VOXEL_GRID_DILATION
    _MLSDilationIterations = 0;     // VOXEL_GRID_DILATION 
    _normalK = 20;
    _gp3Radius = 0.1f;
    _gp3Mu = 5;
}

void MemoryLoc::generateImages()
{
    if (_wordPoints3D.size() == 0)
    {
        getWordCoords();
    }
    if (_wordPoints3D.size() == 0)
    {
        UWARN("No word in MemoryLoc");
    }

    //pcl::PolygonMesh::Ptr mesh = getMesh();
    //pcl::io::savePolygonFilePLY("mesh.ply", *mesh);

    // pick grid locations
    const Signature *s = getSignature(1);
    const Transform &pose = s->getPose();
    const Transform P = pose;
    //std::cout << P << std::endl;
    cv::Mat K = (cv::Mat_<double>(3,3) << 
                525.0f, 0.0f, 320.0f, 
                0.0f, 525.0f, 240.0f, 
                0.0f, 0.0f, 1.0f);
    /*cv::Mat R = (cv::Mat_<double>(3,3) << 
                1.0f, 0.0f, 0.0f, 
                0.0f, 1.0f, 0.0f, 
                0.0f, 0.0f, 1.0f);*/
    cv::Mat R = (cv::Mat_<double>(3,3) <<
                 (double)P.r11(), (double)P.r12(), (double)P.r13(),
                 (double)P.r21(), (double)P.r22(), (double)P.r23(),
                 (double)P.r31(), (double)P.r32(), (double)P.r33());
    cv::Mat rvec(1, 3, CV_64FC1);
    cv::Rodrigues(R, rvec);
    /*cv::Mat tvec = (cv::Mat_<double>(1,3) <<
                   1.0f, 1.0f, 1.0f); */
    cv::Mat tvec = (cv::Mat_<double>(1,3) << 
                    (double)P.x(), (double)P.y(), (double)P.z());

    // generate image using projection
    std::vector<cv::Point2f> planePoints;
    cv::projectPoints(_wordPoints3D, rvec, tvec, K, cv::Mat(), planePoints);

    UWARN("planePoints.size(): %d", planePoints.size());
    // clean up duplicated planePoints for every word
    for (std::map<int, std::vector<long> >::const_iterator iter = _wordToPoint.begin(); 
         iter != _wordToPoint.end(); 
         iter++)
    {
        int wordId = iter->first;
        const std::vector<long> &pointIds = iter->second;
        UWARN("wordId: %d, pointIds.size(): %d", wordId, pointIds.size());
        if (wordId != 7)
        {
            continue;
        }

        size_t elements_num = pointIds.size();
        size_t features_num = 2;
        clustering::DBSCAN::ClusterData dbscanData(elements_num, features_num);
        std::vector<long>::const_iterator idIter = pointIds.begin(); 
        size_t i;
        for (idIter = pointIds.begin(), i = 0; idIter != pointIds.end(); idIter++)
        {
            std::cout << planePoints[*idIter] << std::endl;
            dbscanData(i, 0) = planePoints[*idIter].x;
            dbscanData(i, 1) = planePoints[*idIter].y;
        }
        //cv::KNearest knn(points, trainClasses, 0, false, K );

        //cv::BFMatcher matcher(type==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
        //matcher.knnMatch(descriptors, _dataTree, matches, k);
       
        double eps = 0.00001;
        size_t min_elems = 1;
        clustering::DBSCAN dbs(eps, min_elems);
        dbs.fit(dbscanData);
        std::cout << dbs << std::endl;
    }
  
    UWARN("TEST1");
    ofstream myfile;
    myfile.open("data.txt");
    for (size_t i = 0; i < planePoints.size(); i++)
    {
        if (planePoints[i].x < 0 || planePoints[i].x > 640 || planePoints[i].y < 0 || planePoints[i].y > 480)
        {
            continue;
        }

        if (_pointToWord[i] != 7)
        {
            continue;
        }

        myfile << planePoints[i].x << " " << planePoints[i].y << std::endl;
    }
    myfile.close();
    UWARN("TEST2");

    //save to memory

}

void MemoryLoc::getWordCoords()
{
    std::set<int> ids = getAllSignatureIds();
    for (std::set<int>::const_iterator idIter = ids.begin(); idIter != ids.end(); idIter++)
    {
        const Signature *s = getSignature(*idIter);
        if (!s)
        {
            UWARN("Signature with id %d is empty", *idIter);
            continue;
        }
        
        const Transform &pose = s->getPose();
        //std::cout << "Signature ID: " << *idIter << std::endl;
        //std::cout << "pose: " << pose << std::endl;
        //std::cout << "pose inverse: " << pose.inverse() << std::endl;
        const SensorData &sensorData = s->sensorData();
        const std::vector<CameraModel> &cameraModels = sensorData.cameraModels();
        Transform localTransform;
        if (cameraModels.size() == 0)
        {
            // TODO figure out how to know the local transform
            //Transform tempTransform(0,0,1,0,-1,0,0,0,0,-1,0,0);
            Transform tempTransform(0,0,1,0.105000,-1,0,0,0,0,-1,0,0.431921);
            localTransform = tempTransform;
        }
        else
        {
            const CameraModel &cameraModel = cameraModels[0];
            localTransform = cameraModel.localTransform();
        }

        //const std::multimap<int, cv::KeyPoint> &words2D = s->getWords();
        const std::multimap<int, pcl::PointXYZ> &words3D = s->getWords3();
        for (std::multimap<int, pcl::PointXYZ>::const_iterator pointIter = words3D.begin(); 
             pointIter != words3D.end(); 
             pointIter++)
        {
            /*std::cout << "Word ID: " << pointIter->first << std::endl << "2D Coord: ";
            for (std::multimap<int, cv::KeyPoint>::const_iterator point2Iter = words2D.begin(); 
                 point2Iter != words2D.end(); 
                 point2Iter++)
            {
                if (point2Iter->first == pointIter->first) {
                    std::cout << point2Iter->second.pt << " ";
                }
            }
            std::cout << std::endl;*/

            pcl::PointXYZ localPointPCL = util3d::transformPoint(pointIter->second, localTransform.inverse()); // because words3 is in base_link frame (localTransform applied)) 
            //std::cout << "Local 3D Coord: " << localPointPCL << std::endl;
            pcl::PointXYZ globalPointPCL = util3d::transformPoint(localPointPCL, pose);
            //std::cout << "Global 3D Coord: " << globalPointPCL << std::endl;
            cv::Point3f pointCV(globalPointPCL.x, globalPointPCL.y, globalPointPCL.z);
            _wordPoints3D.push_back(pointCV);
            long pointIdx = _wordPoints3D.size() - 1;
            int wordId = pointIter->first;
            _pointToWord.insert(std::pair<long, int>(pointIdx, wordId));
            _wordToPoint[pointIter->first].push_back(pointIdx);
        }
    }
}

pcl::PolygonMesh::Ptr MemoryLoc::getMesh()
{
    pcl::PolygonMesh::Ptr getMesh();
    std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    std::map<int, Transform> poses;
    std::vector<int> rawCameraIndices;

    getClouds(clouds, poses);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud = assembleClouds(clouds, poses, rawCameraIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rawAssembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*assembledCloud, *rawAssembledCloud);

    assembledCloud = util3d::voxelize(assembledCloud, _voxelSize);
    //pcl::io::savePLYFileASCII("output.ply", *assembledCloud);

    // radius filtering as in MainWindow.cpp
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices = util3d::radiusFiltering(assembledCloud, _filteringRadius, _filteringMinNeighbors);
    pcl::copyPointCloud(*assembledCloud, *indices, *cloudFiltered);
    assembledCloud = cloudFiltered;

    // use MLS to get normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    cloudWithNormals = util3d::mls(
        assembledCloud,
        _MLSRadius,
        _MLSpolygonialOrder,
        _MLSUpsamplingMethod,
        _MLSUpsamplingRadius,
        _MLSUpsamplingStep,
        _MLSPointDensity,
        _MLSDilationVoxelSize,
        _MLSDilationIterations);
    // Re-voxelize to make sure to have uniform density
    cloudWithNormals = util3d::voxelize(cloudWithNormals, _voxelSize);
    
    // adjust normals to view points
    util3d::adjustNormalsToViewPoints(poses, rawAssembledCloud, rawCameraIndices, cloudWithNormals);

    pcl::PolygonMesh::Ptr mesh = util3d::createMesh(cloudWithNormals, _gp3Radius, _gp3Mu);
   
    return mesh;
}

Transform MemoryLoc::computeGlobalVisualTransform(
        const std::vector<int> & oldIds,
        int newId,
        std::string * rejectedMsg,
        int * inliers,
        double * variance) const
{
    bool success = true;

    std::vector<Signature> oldSs;
    for (std::vector<int>::const_iterator it = oldIds.begin() ; it != oldIds.end(); it++)
    {
        if (*it)
        {
            const Signature *oldS = getSignature(*it);
            const Transform &pose = oldS->getPose(); 
            oldSs.push_back(*oldS);
        }
        else
        {
            success = false;
            break;
        }
    }

    const Signature * newS = NULL;
    if (newId)
    {
        newS = getSignature(newId);
    }
    else
    {
        success = false;
    }

    if (success)
    {
        return computeGlobalVisualTransform(oldSs, *newS, rejectedMsg, inliers, variance);
    }
    else
    {
        std::string msg = uFormat("Did not find nodes in oldIds and/or %d", newId);
        if (rejectedMsg)
        {
            *rejectedMsg = msg;
        }
        UWARN(msg.c_str());
    }
    return Transform();
}

Transform MemoryLoc::computeGlobalVisualTransform(
        const std::vector<Signature> & oldSs,
        const Signature & newS,
        std::string * rejectedMsg,
        int * inliersOut,
        double * varianceOut) const
{
    Transform transform;
    std::string msg;
    // Guess transform from visual words

    int inliersCount= 0;
    double variance = 1.0;

    std::multimap<int, pcl::PointXYZ> words3;
    const Transform & basePose = oldSs.begin()->getPose(); 
    for (std::vector<Signature>::const_iterator it1 = oldSs.begin(); it1 != oldSs.end(); it1++)
    {
        Transform relativeT = basePose.inverse() * it1->getPose();
        std::multimap<int, pcl::PointXYZ>::const_iterator it2;
        for (it2 = it1->getWords3().begin(); it2 != it1->getWords3().end(); it2++)
        {
            pcl::PointXYZ globalPoint = util3d::transformPoint(it2->second, relativeT);
            std::multimap<int, pcl::PointXYZ>::iterator it3 = words3.find(it2->first);
            //if (it3 != words3.end())
            //{
            //    std::cout<< "existing point in base frame: " << it3->second << std::endl;
            //    std::cout<< "new point in own frame: " << it2->second << std::endl;
            //    std::cout<< "new point in base frame: " << globalPoint << std::endl << std::endl;
            //    std::cout<< "base pose: " << basePose << std::endl;
            //    std::cout<< "own pose: " << it1->getPose() << std::endl;
            //}
            words3.insert(std::pair<int, pcl::PointXYZ>(it2->first, globalPoint));
        }
    }

    // PnP
    if (!newS.sensorData().stereoCameraModel().isValid() &&
        (newS.sensorData().cameraModels().size() != 1 ||
        !newS.sensorData().cameraModels()[0].isValid()))
    {
        UERROR("Calibrated camera required (multi-cameras not supported).");
    }
    else
    {
        // 3D to 2D
        if ((int)words3.size() >= _bowMinInliers &&
            (int)newS.getWords().size() >= _bowMinInliers)
        {
            UASSERT(newS.sensorData().stereoCameraModel().isValid() || (newS.sensorData().cameraModels().size() == 1 && newS.sensorData().cameraModels()[0].isValid()));
            const CameraModel & cameraModel = newS.sensorData().stereoCameraModel().isValid()?newS.sensorData().stereoCameraModel().left():newS.sensorData().cameraModels()[0];

            std::vector<int> inliersV;
            transform = util3d::estimateMotion3DTo2D(
                    uMultimapToMap(words3),
                    uMultimapToMap(newS.getWords()),
                    cameraModel,
                    _bowMinInliers,
                    _bowIterations,
                    _bowPnPReprojError,
                    _bowPnPFlags,
                    Transform::getIdentity(),
                    uMultimapToMap(newS.getWords3()),
                    &variance,
                    0,
                    &inliersV);
            inliersCount = (int)inliersV.size();
            if (transform.isNull())
            {
                msg = uFormat("Not enough inliers %d/%d between the old signatures and %d",
                        inliersCount, _bowMinInliers, newS.id());
                UINFO(msg.c_str());
            }
            else
            {
                transform = transform.inverse();
            }
        }
        else
        {
            msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)",
                    (int)words3.size(), (int)newS.getWords().size(), _bowMinInliers);
            UINFO(msg.c_str());
        }
    }

    if (!transform.isNull())
    {
        // verify if it is a 180 degree transform, well verify > 90
        float x,y,z, roll,pitch,yaw;
        transform.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
        if (fabs(roll) > CV_PI/2 ||
            fabs(pitch) > CV_PI/2 ||
            fabs(yaw) > CV_PI/2)
        {
            transform.setNull();
            msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)",
                    roll, pitch, yaw);
            UWARN(msg.c_str());
        }
    }

    // transfer to global frame
    transform = basePose * transform.inverse();

    if (rejectedMsg)
    {
        *rejectedMsg = msg;
    }
    if (inliersOut)
    {
        *inliersOut = inliersCount;
    }
    if (varianceOut)
    {
        *varianceOut = variance;
    }
    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    return transform;
}

void MemoryLoc::getClouds(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &clouds, 
                          std::map<int, Transform> &poses)
{
    std::set<int> ids = getAllSignatureIds();
    for (std::set<int>::const_iterator it = ids.begin(); it != ids.end(); it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        SensorData d = getNodeData(*it);
        cv::Mat image, depth;
        d.uncompressData(&image, &depth, 0);
        if (!image.empty() && !depth.empty())
        {
            UASSERT(*it == d.id());
            cloud = util3d::cloudRGBFromSensorData(d);
        }
        else
        {
            UWARN("SensorData missing information");
        }

        if (cloud->size())
        {
            UDEBUG("cloud size: %d", cloud->size());
            clouds.insert(std::make_pair(*it, cloud));
        }
        else
        {
            UWARN("cloud is empty");
        }
        
        const Signature *s = getSignature(*it);
        if (!s)
        {
            UWARN("Signature with id %d is empty", *it);
            continue;
        }
        const Transform &pose = s->getPose();
        poses.insert(std::make_pair(*it, pose));
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MemoryLoc::assembleClouds(const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &clouds, 
                                                                 const std::map<int, Transform> &poses,
                                                                 std::vector<int> &rawCameraIndices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator it1 = clouds.begin();
        it1 != clouds.end();
        it1++)
    {
        std::map<int, Transform>::const_iterator it2 = poses.find(it1->first);
        if (it2 == poses.end())
        {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(it1->second, it2->second);
        *assembledCloud += *transformed;

        rawCameraIndices.resize(assembledCloud->size(), it1->first);
    }
    
    return assembledCloud;
}


} // namespace rtabmap
