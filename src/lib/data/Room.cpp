#include "lib/data/Room.h"
#include <cassert>

Room::Room(int id, const std::map<int, Word> &words) : _id(id), _words(words) {}

int Room::getId() const { return _id; }

void Room::addWordIds(std::vector<int> &&wordIds) {
  _wordIds.insert(wordIds.begin(), wordIds.end());
}
