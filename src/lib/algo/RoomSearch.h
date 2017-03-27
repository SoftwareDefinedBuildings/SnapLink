#pragma once

#include "lib/data/Room.h"
#include "lib/data/Word.h"
#include <memory>

class RoomSearch final {
public:
  explicit RoomSearch(const std::map<int, Room> &rooms,
                      const std::map<int, Word> &words);

  /**
   * return the id of the most similar database.
   */
  int search(const std::vector<int> &wordIds) const;

private:
  const std::map<int, Room> &_rooms;
  const std::map<int, Word> &_words;
};
