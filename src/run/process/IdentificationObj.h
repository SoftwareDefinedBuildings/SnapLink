#pragma once

#include "lib/algo/Feature.h"
#include "lib/algo/Perspective.h"
#include "lib/algo/Visibility.h"
#include "lib/algo/WordSearch.h"
#include "lib/algo/DbSearch.h"
#include <QEvent>
#include <QObject>


class BWFrontEndObj;
class HTTPFrontEndObj;
class CameraModel;
class Session;

class IdentificationObj final : public QObject {
public:
  explicit IdentificationObj(const std::shared_ptr<Words> &words,
                 std::unique_ptr<Labels> &&labels, int sampleSize, int corrSize, double distRatio);
  ~IdentificationObj();

  void setHTTPFrontEndObj(std::shared_ptr<HTTPFrontEndObj> httpFrontEndObj);
  void setBWFrontEndObj(std::shared_ptr<BWFrontEndObj> bwFrontEndObj);
  bool identify(const cv::Mat &image, const CameraModel &camera,
                std::vector<std::string> &names, Session &session);

protected:
  bool event(QEvent *event);

private:
  std::shared_ptr<HTTPFrontEndObj> _httpFrontEndObj;
  std::shared_ptr<BWFrontEndObj> _bwFrontEndObj;
  Feature _feature;
  WordSearch _wordSearch;
  DbSearch _dbSearch;
  Perspective _perspective;
  Visibility _visibility;
};
