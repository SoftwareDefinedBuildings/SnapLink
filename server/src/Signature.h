#pragma once

#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <list>
#include <vector>
#include <set>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>

class Signature
{
public:
    Signature(int id,
              int mapId,
              int dbId,
              const rtabmap::Transform &pose,
              const rtabmap::SensorData &sensorData);
    virtual ~Signature();

    int getId() const;
    int getMapId() const;
    int getDbId() const;
    const rtabmap::Transform &getPose() const;
    const rtabmap::SensorData &getSensorData() const;
    const std::multimap<int, cv::KeyPoint> &getWords() const;
    const std::multimap<int, cv::Point3f> &getWords3D() const;
    const std::multimap<int, cv::Mat> &getWordsDescriptors() const;

    /**
     * ownership transfer
     */
    void setWords(const std::multimap<int, cv::KeyPoint> &words);
    void setWords3D(const std::multimap<int, cv::Point3f> &words3D);
    void setWordsDescriptors(const std::multimap<int, cv::Mat> &descriptors);

private:
    int _id;
    int _mapId;
    int _dbId;
    rtabmap::Transform _pose;
    rtabmap::SensorData _sensorData;

    // Contains all words (Some can be duplicates -> if a word appears 2
    // times in the signature, it will be 2 times in this list)
    // Words match with the CvSeq keypoints and descriptors
    std::multimap<int, cv::KeyPoint> _words; // word <id, keypoint>
    std::multimap<int, cv::Point3f> _words3D; // word <id, 3D point> // in base_link frame (localTransform applied))
    std::multimap<int, cv::Mat> _wordsDescriptors;
};

