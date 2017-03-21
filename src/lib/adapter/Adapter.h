#pragma once

#include "lib/data/Label.h"
#include "lib/data/Word.h"
#include "lib/data/Room.h"
#include "lib/data/Image.h"
#include <set>
#include <map>
#include <list>

class Adapter {
public:
  virtual ~Adapter() = default;

  virtual bool init(const std::set<std::string> &dbPaths) = 0;

  /**
   * room ID : vector of images, indexed by image ID
   */
  virtual const std::map<int, std::vector<Image>> &getImages() const = 0;

  /**
   * word ID : word
   */
  virtual const std::map<int, Word> &getWords() const = 0;

  /**
   * room ID : room
   */
  virtual const std::map<int, Room> &getRooms() const = 0;

  /**
   * room ID : vector of labels
   */
  virtual const std::map<int, std::vector<Label>> &getLabels() const = 0;
};
