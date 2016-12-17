#pragma once

enum SessionType { HTTP_POST = 0, BOSSWAVE = 1 };

class Session {
public:
  long id;
  SessionType type;
  QSemaphore detected;
  std::vector<std::string> names;

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
