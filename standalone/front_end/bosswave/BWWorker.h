#pragma once

#include <libbw.h>
#include <QSemaphore>
#include <string.h>
#include "data/Session.h"
#include <memory>
#include <vector>
#include <sstream>
#include <mutex>
#include <random>
#include <opencv2/core/core.hpp>
#include <QCoreApplication>
#include "util/Time.h"
#include "event/QueryEvent.h"
#include "process/IdentificationObj.h"
#include "data/CameraModel.h"
#include <cstdlib>
#include <strings.h>

#define IMAGE_INIT_SIZE 100000
#define BW_MSG_LENGTH 9

class CameraModel;
class FeatureStage;

typedef struct {
  QSemaphore detected;
  std::unique_ptr<std::vector<std::string>> names;
  std::unique_ptr<Session> session;
} BWConnectionInfo;


class BWWorker : public QObject
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
           IdentificationObj * id, 
           std::map<long, BWConnectionInfo *> *map,
           std::uniform_int_distribution<unsigned long long> *dis,
           std::mt19937 *gen,
           std::mutex *mutex,
           unsigned int *numClients
           );

  private:
  void workComplete(BWConnectionInfo *connInfo);
  static void createData(const std::vector<char> &data, double fx, double fy,
                                      double cx, double cy, cv::Mat &image,
                                      CameraModel &camera);
  PMessage _msg;
  IdentificationObj *_identObj;
  std::map<long, BWConnectionInfo *> *_connInfoMap;
  std::uniform_int_distribution<unsigned long long> *_dis;
  std::mt19937 *_gen;
  std::mutex *_mutex;
  unsigned int *_numClients;
};
