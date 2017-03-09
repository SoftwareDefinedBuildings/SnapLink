#include "lib/data/Room.h"
#include <cassert>

Room::Room(int id) : _id(id) {}

int Room::getId() const { return _id; }

void Room::addWordIds(std::vector<int> &&wordIds) {
  _wordIds.insert(wordIds.begin(), wordIds.end());
}

const std::set<int> &Room::getWordIds() const { return _wordIds; }
