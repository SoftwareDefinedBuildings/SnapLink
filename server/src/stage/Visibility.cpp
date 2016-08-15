#include "stage/Visibility.h"
#include "data/PerfData.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/LocationEvent.h"
#include "util/Utility.h"
#include <QCoreApplication>
#include <QDebug>
#include <QDirIterator>
#include <QTextStream>
#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <pcl/point_types.h>

Visibility::Visibility() : _httpServer(nullptr) {}

Visibility::~Visibility() { _httpServer = nullptr; }

void Visibility::putLabels(std::unique_ptr<Labels> &&labels) {
  _labels = std::move(labels);
}

void Visibility::setHTTPServer(HTTPServer *httpServer) {
  _httpServer = httpServer;
}

bool Visibility::event(QEvent *event) {
  if (event->type() == LocationEvent::type()) {
    LocationEvent *locEvent = static_cast<LocationEvent *>(event);
    std::unique_ptr<SensorData> sensorData = locEvent->takeSensorData();
    std::unique_ptr<Transform> pose = locEvent->takePose();
    std::unique_ptr<PerfData> perfData = locEvent->takePerfData();
    const void *session = locEvent->getSession();
    std::unique_ptr<std::vector<std::string>> names(
        new std::vector<std::string>());
    *names = process(locEvent->dbId(), *sensorData, *pose);
    if (!names->empty()) {
      QCoreApplication::postEvent(
          _httpServer,
          new DetectionEvent(std::move(names), std::move(perfData), session));
    } else {
      QCoreApplication::postEvent(_httpServer, new FailureEvent(session));
    }
    return true;
  }
  return QObject::event(event);
}

std::vector<std::string> Visibility::process(int dbId,
                                             const SensorData &sensorData,
                                             const Transform &pose) const {
  const std::list<std::unique_ptr<Label>> &labels =
      _labels->getLabels().at(dbId);
  std::vector<cv::Point3f> points;
  std::vector<std::string> names;
  for (auto &label : labels) {
    points.emplace_back(label->getPoint3());
    names.emplace_back(label->getName());
  }

  std::vector<std::string> results;

  qDebug() << "processing transform = " << pose.prettyPrint().c_str();

  std::vector<cv::Point2f> planePoints;
  std::vector<std::string> visibleLabels;

  const CameraModel &model = sensorData.getCameraModel();
  cv::Mat K = model.K();
  Transform P = pose;
  cv::Mat R =
      (cv::Mat_<double>(3, 3) << (double)P.r11(), (double)P.r12(),
       (double)P.r13(), (double)P.r21(), (double)P.r22(), (double)P.r23(),
       (double)P.r31(), (double)P.r32(), (double)P.r33());
  cv::Mat rvec(1, 3, CV_64FC1);
  cv::Rodrigues(R, rvec);
  cv::Mat tvec =
      (cv::Mat_<double>(1, 3) << (double)P.x(), (double)P.y(), (double)P.z());

  // do the projection
  cv::projectPoints(points, rvec, tvec, K, cv::Mat(), planePoints);

  // find points in the image
  int cols = sensorData.getImage().cols;
  int rows = sensorData.getImage().rows;
  std::map<std::string, std::vector<double>> distances;
  std::map<std::string, std::vector<cv::Point2f>> labelPoints;
  cv::Point2f center(cols / 2, rows / 2);

  for (unsigned int i = 0; i < points.size(); ++i) {
    std::string name = names[i];
    // if (uIsInBounds(int(planePoints[i].x), 0, cols) &&
    //        uIsInBounds(int(planePoints[i].y), 0, rows))
    if (true) {
      if (Utility::isInFrontOfCamera(points[i], P)) {
        visibleLabels.emplace_back(name);
        double dist = cv::norm(planePoints[i] - center);
        distances[name].emplace_back(dist);
        labelPoints[name].emplace_back(planePoints[i]);
        qDebug() << "Find label " << name.c_str() << " at (" << planePoints[i].x
                 << "," << planePoints[i].y << ")";
      } else {
        qDebug() << "Label " << name.c_str() << " invalid at ("
                 << planePoints[i].x << "," << planePoints[i].y << ")"
                 << " because it is from the back of the camera";
      }
    } else {
      qDebug() << "label " << name.c_str() << " invalid at ("
               << planePoints[i].x << "," << planePoints[i].y << ")";
    }
  }

  if (!distances.empty()) {
    // find the label with minimum mean distance
    std::pair<std::string, std::vector<double>> minDist =
        *min_element(distances.begin(), distances.end(), CompareMeanDist());
    std::string minlabel = minDist.first;
    std::cout << "Nearest label " << minlabel << " with mean distance "
              << CompareMeanDist::meanDist(minDist.second) << std::endl;
    results.emplace_back(minlabel);
  } else {
    std::cout << "No label is qualified" << std::endl;
  }
  return results;
}

double CompareMeanDist::meanDist(const std::vector<double> &vec) {
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  double mean = sum / vec.size();
  return mean;
}

bool CompareMeanDist::operator()(const PairType &left,
                                 const PairType &right) const {
  return meanDist(left.second) < meanDist(right.second);
}
