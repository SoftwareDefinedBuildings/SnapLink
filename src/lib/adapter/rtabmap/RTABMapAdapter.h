#pragma once

#include <list>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <pcl/filters/extract_indices.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/VisualWord.h>
#include <set>
#include <typeinfo>

class RTABMapAdapter final : public Adapter {
public:
  explicit RTABMapAdapter();

  // read data from database files
  bool init(const std::set<std::string> &dbPaths) final;

  const std::map<int, std::vector<Image>> &getImages() const final;
  const std::map<int, Word> &getWords() const final;
  const std::map<int, Room> &getRooms() const final;
  const std::map<int, std::vector<Label>> &getLabels() const final;

private:
  std::vector<Image> readRoomImages(const std::string &dbPath);
  static std::vector<Label> readRoomLabels(const std::string &dbPath, const std::vector<Image> &roomImages);

  // RTABMap requires to use an graph optimizer to obtain the correct poses of each image
  static std::map<int, Transform> computePoses(const std::string &dbPath);

  static std::map<int, Word> createWords(
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);
  static std::map<int, Room> createRooms(const std::map<int, Word> &words);

  static bool getPoint3World(const rtabmap::Signature &signature,
                             const cv::Point2f &point2, pcl::PointXYZ &point3);

private:
  int _nextImageId;
  std::map<int, std::vector<Image>> _images;
  std::map<int, Word> _words;
  std::map<int, Room> _rooms;
  std::map<int, std::vector<Label>> _labels;
};
