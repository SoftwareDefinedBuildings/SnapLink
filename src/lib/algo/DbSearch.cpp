#include "algo/DbSearch.h"
#include <cassert>
#include <iostream>

DbSearch::DbSearch(const std::shared_ptr<Words> &words) : _words(words) {}

int DbSearch::search(const std::vector<int> &wordIds) const {
  const auto &words = _words->getWordsById();

  std::map<int, int> dbCounts; // dbId: number of shared words in db
  for (auto wordId : wordIds) {
    const auto iter = words.find(wordId);
    assert(iter != words.end());
    const std::shared_ptr<Word> &word = iter->second;

    const auto &points3Map = word->getPoints3Map();
    for (const auto &points3 : points3Map) {
      int dbId = points3.first;
      auto jter = dbCounts.find(dbId);
      if (jter == dbCounts.end()) {
        auto ret = dbCounts.emplace(dbId, 0);
        jter = ret.first;
      }
      jter->second++; // TODO: +1 or +points3.size() ?
    }
  }

  const auto &wordsByDb = _words->getWordsByDb();
  std::map<int, double> dbSims; // dbId: similarity
  for (const auto &count : dbCounts) {
    int dbId = count.first;
    double sim = static_cast<double>(count.second) / wordsByDb.at(dbId).size();
    dbSims[dbId] = sim;
    std::cout << "dbId = " << dbId << ", similarity = " << sim << std::endl;
  }

  auto maxCount = std::max_element(
      dbSims.begin(), dbSims.end(),
      [](const std::pair<int, double> &p1, const std::pair<int, double> &p2) {
        return p1.second < p2.second;
      });

  int dbId = maxCount->first;
  std::cout << "max dbId = " << dbId << std::endl;

  return dbId;
}
