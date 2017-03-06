#pragma once

#include <memory>
#include <QSemaphore>

class FrontEndWrapper;

class Session final {
public:
  long id;
  // a pointer to the front end wrapper that initiates the session
  std::shared_ptr<FrontEndWrapper> frontEndWrapper;
  std::unique_ptr<std::vector<std::string>> results;
  QSemaphore detected;

  // timing values
public:
  unsigned long long overallStart;
  unsigned long long overallEnd;
  unsigned long long featuresStart;
  unsigned long long featuresEnd;
  unsigned long long wordSearchStart;
  unsigned long long wordSearchEnd;
  unsigned long long dbSearchStart;
  unsigned long long dbSearchEnd;
  unsigned long long perspectiveStart;
  unsigned long long perspectiveEnd;
  unsigned long long visibilityStart;
  unsigned long long visibilityEnd;
};
