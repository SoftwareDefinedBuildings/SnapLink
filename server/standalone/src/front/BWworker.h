#include "front/BW.h"
#include "data/CameraModel.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "process/Identification.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <cstdlib>
#include <string.h>
#include <strings.h>
#include "data/Session.h"
#include <QObject>
#include <QSemaphore>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <random>
#include <QCoreApplication>
#include </root/workspace/CellMate/server/third_party/qtlibbw/libbw.h>
#include <allocations.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <QThread>


#define BW_MSG_LENGTH 9
#define DEFAULT_CHANNEL "scratch.ns/cellmate"


class CameraModel;
class FeatureStage;


typedef struct {
  double fx;
  double fy;
  double cx;
  double cy;
} CameraInfo;

typedef struct {
  QSemaphore detected;
  std::unique_ptr<std::vector<std::string>> names;
  std::unique_ptr<Session> session;
} ConnectionInfo;



class BWServer;
class BWWorker : public QObject
{ 
  Q_OBJECT
  public:
    BWWorker(){};
    virtual ~BWWorker(){};
public slots:
    void doWork(const PMessage &msg, BWServer *server);
signals:
    void doneWork(std::string result, std::string identity){};
  private:
    void workComplete(BWServer *server, ConnectionInfo *connInfo);
};


