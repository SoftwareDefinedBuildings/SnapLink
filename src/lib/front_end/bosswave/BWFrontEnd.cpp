#include "lib/front_end/bosswave/BWFrontEnd.h"

BWFrontEnd::BWFrontEnd() : _numClients(0), _bw(BW::instance()) {}

BWFrontEnd::~BWFrontEnd() {
  stop();
  _numClients = 0;
}

void BWFrontEnd::start(unsigned int maxClients) {
  _maxClients = maxClients;
  _thread.reset(new QThread());
  this->moveToThread(_thread.get());
  connect(_thread.get(), &QThread::started, this, &BWFrontEnd::run);
  _thread->start();
}

void BWFrontEnd::stop() { _bw.reset(); }

void BWFrontEnd::registerOnQuery(std::function<std::vector<std::string>(
                                     std::unique_ptr<cv::Mat> &&image,
                                     std::unique_ptr<CameraModel> &&camera)>
                                     onQuery) {
  _onQuery = onQuery;
}

void BWFrontEnd::run() {
  QObject::connect(_bw.get(), &BW::agentChanged, this,
                   &BWFrontEnd::agentChanged);
  _entity = getEntity();
  _bw->connectAgent(_entity);
  _bw->setEntity(_entity, [](QString err, QString vk) {});
}

void BWFrontEnd::agentChanged(bool success, QString msg) {
  _bw->subscribe(
      DEFAULT_CHANNEL, QString(), true, QList<RoutingObject *>(), QDateTime(),
      -1, QString(), false, false,
      std::bind(&BWFrontEnd::parseMessage, this, std::placeholders::_1));
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

QByteArray BWFrontEnd::getEntity() {
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

void BWFrontEnd::parseMessage(PMessage msg) {
  std::cerr << "DEBUG:: received message" << std::endl;
  QThread *workerThread = new QThread();
  BWWorker *worker = new BWWorker(msg, _onQuery, &_numClients);
  worker->moveToThread(workerThread);
  QObject::connect(this, &BWFrontEnd::askWorkerDoWork, worker,
                   &BWWorker::doWork);
  QObject::connect(worker, &BWWorker::doneWork, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::doneWork, worker, &BWWorker::deleteLater);
  QObject::connect(workerThread, &QThread::finished, workerThread,
                   &QThread::deleteLater);
  QObject::connect(worker, &BWWorker::error, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::error, worker, &BWWorker::deleteLater);
  QObject::connect(worker, &BWWorker::error, this,
                   &BWFrontEnd::workerReturnError);
  QObject::connect(worker, &BWWorker::doneWork, this,
                   &BWFrontEnd::publishResult);
  workerThread->start();
  _numClients++;
  emit this->askWorkerDoWork();
}
