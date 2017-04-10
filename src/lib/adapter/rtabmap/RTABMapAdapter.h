#pragma once

#include "lib/adapter/Adapter.h"
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
#include <sqlite3.h>

class RTABMapAdapter final : public Adapter {
public:
  explicit RTABMapAdapter();

  // read data from database files
  bool init(const std::set<std::string> &dbPaths) final;

  const std::map<int, std::vector<Image>> &getImages() final;
  const std::map<int, Word> &getWords() final;
  const std::map<int, Room> &getRooms() final;
  const std::map<int, std::vector<Label>> &getLabels() final;
  bool addLabel(std::string label_name, std::string label_id, std::string label_x, std::string label_y);
  bool getLabels(std::vector<int> &imageIds,
                 std::vector<int> &xList,
                 std::vector<int> &yList,
                 std::vector<std::string> &labels);
  bool createLabelTable();
  void closeLabelDB();
private:
  std::vector<Image> readRoomImages(const std::string &dbPath, int roomId);
  std::vector<Label> readRoomLabels(const std::string &dbPath, int roomId);

  void createWords();
  void createRooms();


private:
  int _nextImageId;
  // {room ID : {signature ID in database : image ID in memory}}
  std::map<int, std::map<int, int>> _sigImageIdMap;
  std::map<int, std::vector<Image>> _images;
  std::map<int, Word> _words;
  std::map<int, Room> _rooms;
  std::map<int, std::vector<Label>> _labels;
  sqlite3 *_labelDB;
  std::string _labelPath;
};
