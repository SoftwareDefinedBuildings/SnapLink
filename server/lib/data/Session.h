#pragma once

#include <boost/uuid/uuid.hpp>

enum SessionType { HTTP_POST = 0, BOSSWAVE = 1 };

class Session {
public:
  boost::uuids::uuid id;
  SessionType type;

public:
  long overallStart;
  long overallEnd;
  long featuresStart;
  long featuresEnd;
  long wordsStart;
  long wordsEnd;
  long signaturesStart;
  long signaturesEnd;
  long perspectiveStart;
  long perspectiveEnd;
};
