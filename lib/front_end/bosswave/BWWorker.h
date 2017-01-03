#pragma once

#include <libbw.h>
#include <atomic>
#include <string.h>
#include <memory>
#include <vector>
#include <sstream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <QCoreApplication>
#include "data/CameraModel.h"
#include <cstdlib>
#include <strings.h>

#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000
#define BW_MSG_LENGTH 9

class CameraModel;

class BWWorker final : public QObject
{
  Q_OBJECT
public slots:
void doWork();
signals:
  void finished();
  void error();
  void doneWork(QString result, QString identity);
  public:
  BWWorker(PMessage message,
           std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
                      std::unique_ptr<CameraModel> &&camera)> onQuery,
           std::atomic<unsigned int> *_numClients
           );

  private:
  // void workComplete(BWConnectionInfo *connInfo);
  static void createData(const std::vector<char> &data, double fx, double fy,
                                      double cx, double cy, cv::Mat &image,
                                      CameraModel &camera);
  PMessage _msg;
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> _onQuery;
  std::atomic<unsigned int> *_numClients;
};
