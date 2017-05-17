#pragma once

#include "lib/data/Image.h"
#include "lib/data/Label.h"
#include "lib/data/Room.h"
#include "lib/data/Word.h"
#include <list>
#include <map>
#include <set>

class Adapter {
public:
  virtual ~Adapter() = default;

  virtual bool init(const std::set<std::string> &dbPaths) = 0;

  /**
   * room ID : vector of images, indexed by image ID
   */
  virtual const std::map<int, std::vector<Image>> &getImages() = 0;

  /**
   * word ID : word
   */
  virtual const std::map<int, Word> &getWords() = 0;

  /**
   * room ID : room
   */
  virtual const std::map<int, Room> &getRooms() = 0;

  /**
   * room ID : vector of labels
   */
  virtual const std::map<int, std::vector<Label>> &getLabels() = 0;

  virtual bool putLabel(int roomId, std::string, std::string, std::string, std::string) = 0;
};
