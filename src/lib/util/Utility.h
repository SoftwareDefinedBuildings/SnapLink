#pragma once

#include <list>
#include <map>
#include <opencv2/core/core.hpp>
#include <pcl/common/common.h>
#include <vector>

class Image;
class Transform;

class Utility final {
public:
  /* Returns current time in milliseconds. */
  static unsigned long long getTime();
  static bool qrExtract(const cv::Mat &image, std::vector<std::string> &results);
  static bool getPoint3World(const Image &image, const cv::Point2f &point2,
                             cv::Point3f &point3);

  static bool compareCVPoint2f(cv::Point2f p1, cv::Point2f p2);

  static bool isInFrontOfCamera(const cv::Point3f &point,
                                const Transform &pose);

  template <class K, class V>
  static std::vector<K> Keys(const std::map<K, V> &m) {
    std::vector<K> v(m.size());
    int i = 0;
    for (const auto &entry : m) {
      v[i] = entry.first;
      ++i;
    }
    return v;
  }

  template <class K, class V>
  static std::map<K, V> MultimapToMapUnique(const std::multimap<K, V> &m) {
    std::map<K, V> mapOut;
    std::list<K> uniqueKeys = Utility::UniqueKeys(m);
    for (const auto &key : uniqueKeys) {
      if (m.count(key) == 1) {
        const auto iter = m.find(key);
        mapOut.insert(mapOut.end(), std::pair<K, V>(iter->first, iter->second));
      }
    }
    return mapOut;
  }

  template <class K, class V>
  static std::list<K> UniqueKeys(const std::multimap<K, V> &mm) {
    std::list<K> l;
    typename std::list<K>::reverse_iterator lastValue;
    for (typename std::multimap<K, V>::const_iterator iter = mm.begin();
         iter != mm.end(); ++iter) {
      if (iter == mm.begin() ||
          (iter != mm.begin() && *lastValue != iter->first)) {
        l.push_back(iter->first);
        lastValue = l.rbegin();
      }
    }
    return l;
  }
};
