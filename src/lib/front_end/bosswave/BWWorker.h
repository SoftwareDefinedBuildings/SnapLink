#pragma once

#include <libbw.h>
#include <string.h>
#include <memory>
#include <vector>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <QCoreApplication>
#include "lib/data/CameraModel.h"
#include <cstdlib>
#include <strings.h>

#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000
#define BW_MSG_LENGTH 9

class BWWorker final : public QObject
{
  Q_OBJECT

public:
  explicit BWWorker(PMessage message,
           std::function<std::vector<std::string>(const cv::Mat &image, const CameraModel &camera)> onQuery,
           std::atomic<unsigned int> &_numClients);

public slots:
  void process();

signals:
  void error();
  void done(QString result, QString identity);

private:
  static void createData(const std::vector<char> &data, double fx, double fy,
                                      double cx, double cy, cv::Mat &image,
                                      CameraModel &camera);

private:
  static const std::string none;
  PMessage _msg;
  std::function<std::vector<std::string>(const cv::Mat &image, const CameraModel &camera)> _onQuery;
  std::atomic<unsigned int> &_numClients;
};
