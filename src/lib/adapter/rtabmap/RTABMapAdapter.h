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

#define DIST_RATIO 0.7

class RTABMapAdapter final : public Adapter {
public:
  explicit RTABMapAdapter(float distRatio = DIST_RATIO);

  // read data from database files
  bool init(const std::set<std::string> &dbPaths) final;

  const std::map<int, std::vector<Image>> &getImages() final;
  const std::map<int, Word> &getWords() final;
  const std::map<int, Room> &getRooms() final;
  const std::map<int, std::vector<Label>> &getLabels() final;
  bool putLabel(int roomId,
                        std::string label_name, 
                        std::string label_id, 
                        std::string label_x, 
                        std::string label_y) final;
private:
  std::vector<Image> readRoomImages(const std::string &dbPath, int roomId);
  std::vector<Label> readRoomLabels(const std::string &dbPath, int roomId);

  sqlite3 *createLabelTable(int roomId);
  void createWords();
  void createRooms();


private:
  int _nextImageId;
  float _distRatio;
  // {room ID : {signature ID in database : image ID in memory}}
  std::map<int, std::map<int, int>> _sigImageIdMap;
  // {room ID : {image ID in memory : signature ID in database}}
  std::map<int, std::map<int, int>> _imageSigIdMap;
  std::map<int, std::vector<Image>> _images;
  std::map<int, Word> _words;
  std::map<int, Room> _rooms;
  std::map<int, std::vector<Label>> _labels;
  std::map<int, std::string> _roomPaths;
  sqlite3 *_labelDB;
  std::string _labelPath;
};
