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

class RTABMapAdapter final : public Adapter {
public:
  explicit RTABMapAdapter();

  // read data from database files
  bool init(const std::set<std::string> &dbPaths) final;

  const std::map<int, std::vector<Image>> &getImages() const final;
  const std::map<int, Word> &getWords() const final;
  const std::map<int, Room> &getRooms() const final;
  const std::map<int, std::vector<Label>> &getLabels() const final;
  // TODO addLabels()

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
};
