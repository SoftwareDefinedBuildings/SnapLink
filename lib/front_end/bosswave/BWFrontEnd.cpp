#include "front_end/bosswave/BWFrontEnd.h"

BWFrontEnd::BWFrontEnd() {
  _numClients = 0;
  _bw = BW::instance();
}

BWFrontEnd::~BWFrontEnd() {
  stop();
  _numClients = 0;
}

void BWFrontEnd::startRun() {
  qDebug() << "starting run";
  _maxClients = MAX_CLIENTS;
  QObject::connect(_bw, &BW::agentChanged, this, &BWFrontEnd::agentChanged);
  _entity = mustGetEntity();
  _bw->connectAgent(_entity);
  _bw->setEntity(_entity, [](QString err, QString vk) {});
}

void BWFrontEnd::parseMessage(PMessage msg) {
  QThread *workerThread = new QThread();
  BWWorker *worker = new BWWorker(msg, &_onQuery, &_numClients);
  worker->moveToThread(workerThread);
  QObject::connect(this, &BWFrontEnd::askWorkerDoWork, worker, &BWWorker::doWork);
  QObject::connect(worker, &BWWorker::doneWork, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::doneWork, worker, &BWWorker::deleteLater);
  QObject::connect(workerThread, &QThread::finished, workerThread,
                   &QThread::deleteLater);
  QObject::connect(worker, &BWWorker::error, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::error, worker, &BWWorker::deleteLater);
  QObject::connect(worker, &BWWorker::error, this,
                   &BWFrontEnd::workerReturnError);
  QObject::connect(worker, &BWWorker::doneWork, this, &BWFrontEnd::publishResult);
  workerThread->start();
  _numClients++;
  emit this->askWorkerDoWork();
}

void BWFrontEnd::agentChanged() {
  _bw->subscribe(DEFAULT_CHANNEL, QString(), true, QList<RoutingObject *>(),
                 QDateTime(), -1, QString(), false, false,
                 [&](PMessage msg) { this->parseMessage(msg); });
}

QByteArray BWFrontEnd::mustGetEntity() {
  QString entitypath;
  // Try environment variable
  QByteArray a = qgetenv("BW2_DEFAULT_ENTITY");
  if (a.length() != 0) {
    entitypath = a.data();
  } else {
    // Try local file
    entitypath = "entity.ent";
  }
  QFile f(entitypath);
  if (!f.open(QIODevice::ReadOnly)) {
    qDebug() << "could not open entity file";
    return QByteArray();
  }
  QByteArray contents = f.readAll().mid(1);
  return contents;
}

void BWFrontEnd::publishResult(QString result, QString identity) {
  auto ponum = bwpo::num::Text;
  QString msg = result;
  std::string channel = DEFAULT_CHANNEL;
  _bw->publishText(QString::fromStdString(channel) + "/" + identity, QString(),
                   true, QList<RoutingObject *>(), ponum, msg, QDateTime(), -1,
                   "partial", false, false, [](QString err) {
                     if (!err.isEmpty()) {
                       qDebug() << "publish error: " << err;
                     } else {
                       qDebug() << "published ok";
                     }
                   });
  _numClients--;
}



void BWFrontEnd::workerReturnError() {
  // publish error message to identity channel
  _numClients--;
}

bool BWFrontEnd::start(unsigned int maxClients) {
  _maxClients = maxClients;

  // start BW instance, subscribe the appropriate channel with assigned call back function
  startRun();

  return true;
}

void BWFrontEnd::stop() {
  free(_bw);
  _bw = nullptr;
}

void BWFrontEnd::registerOnQuery(std::function<std::vector<std::string>(
                                       std::unique_ptr<cv::Mat> &&image,
                                       std::unique_ptr<CameraModel> &&camera)>
                                       onQuery) {
  _onQuery = onQuery;
}
