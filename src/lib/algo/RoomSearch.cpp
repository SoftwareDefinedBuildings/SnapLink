#include "lib/algo/RoomSearch.h"
#include <cassert>
#include <iostream>

RoomSearch::RoomSearch(const std::map<int, Room> &rooms,
                       const std::map<int, Word> &words)
    : _rooms(rooms), _words(words) {}

int RoomSearch::search(const std::vector<int> &wordIds) const {
  std::map<int, int> counts; // roomId: number of shared words in room
  for (auto wordId : wordIds) {
    const auto iter = _words.find(wordId);
    assert(iter != _words.end());
    const Word &word = iter->second;

    const auto &points3Map = word.getPoints3Map();
    for (const auto &points3 : points3Map) {
      int roomId = points3.first;
      auto jter = counts.find(roomId);
      if (jter == counts.end()) {
        auto ret = counts.emplace(roomId, 0);
        jter = ret.first;
      }
      jter->second++; // TODO: +1 or +points3.size() ?
    }
  }

  std::map<int, double> sims; // roomId: similarity
  for (const auto &count : counts) {
    int roomId = count.first;
    double sim = static_cast<double>(count.second) /
                 _rooms.at(roomId).getWordIds().size();
    sims[roomId] = sim;
    std::cerr << "roomId = " << roomId << ", similarity = " << sim << std::endl;
  }

  auto max = std::max_element(
      sims.begin(), sims.end(),
      [](const std::pair<int, double> &p1, const std::pair<int, double> &p2) {
        return p1.second < p2.second;
      });

  int roomId = max->first;
  std::cerr << "max roomId = " << roomId << std::endl;

  return roomId;
}
