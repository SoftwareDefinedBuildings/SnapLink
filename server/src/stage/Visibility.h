#pragma once

#include "data/Labels.h"
#include <QObject>
#include <memory>
#include <numeric>

class Transform;
class CameraModel;
class HTTPServer;

class Visibility : public QObject {
public:
  Visibility();
  virtual ~Visibility();

  void putLabels(std::unique_ptr<Labels> &&labels);
  void setHTTPServer(HTTPServer *httpServer);

protected:
  virtual bool event(QEvent *event);

private:
  std::vector<std::string> process(int dbId, const CameraModel &camera,
                                   const Transform &pose) const;

private:
  std::unique_ptr<Labels> _labels;
  HTTPServer *_httpServer;
};

struct CompareMeanDist {
  typedef std::pair<std::string, std::vector<double>> PairType;

  static double meanDist(const std::vector<double> &vec);
  bool operator()(const PairType &left, const PairType &right) const;
};
