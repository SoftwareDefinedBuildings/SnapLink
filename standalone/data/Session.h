#pragma once

#include <memory>
#include <QSemaphore>

enum SessionType { HTTP_POST = 0, BOSSWAVE = 1 };

class Session final {
public:
  long id;
  SessionType type;

public:
  unsigned long long overallStart;
  unsigned long long overallEnd;
  unsigned long long featuresStart;
  unsigned long long featuresEnd;
  unsigned long long wordsStart;
  unsigned long long wordsEnd;
  unsigned long long perspectiveStart;
  unsigned long long perspectiveEnd;
};
