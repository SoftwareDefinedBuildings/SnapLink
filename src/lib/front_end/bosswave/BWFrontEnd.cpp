#include "lib/front_end/bosswave/BWFrontEnd.h"

BWFrontEnd::BWFrontEnd() : _numClients(0), _bw(BW::instance()) {}

BWFrontEnd::~BWFrontEnd() {
  stop();
  _bw.reset();
  _numClients = 0;
}

void BWFrontEnd::start(const std::string &uri, unsigned int maxClients) {
  _uri = uri;
  _maxClients = maxClients;
  _thread.reset(new QThread());
  this->moveToThread(_thread.get());
  connect(_thread.get(), &QThread::started, this, &BWFrontEnd::run);
  _thread->start();
}

void BWFrontEnd::stop() {
  _thread->exit();
  _thread.reset();
}

void BWFrontEnd::registerOnQuery(std::function<std::vector<std::string>(
                                     std::unique_ptr<cv::Mat> &&image,
                                     std::unique_ptr<CameraModel> &&camera)>
                                     onQuery) {
  _onQuery = onQuery;
}

void BWFrontEnd::run() {
  connect(_bw.get(), &BW::agentChanged, this, &BWFrontEnd::agentChanged);
  _entity = getEntity();
  _bw->connectAgent(_entity);
  _bw->setEntity(_entity, [](QString err, QString vk) {});
}

void BWFrontEnd::agentChanged(bool success, QString msg) {
  _bw->subscribe(
      QString::fromStdString(_uri), QString(), true, QList<RoutingObject *>(),
      QDateTime(), -1, QString(), false, false,
      std::bind(&BWFrontEnd::onMessage, this, std::placeholders::_1));
}

void BWFrontEnd::respond(QString result, QString identity) {
  auto ponum = bwpo::num::Text;
  // TODO this uri needs to be contained in the query messsage later
  QString uri = QString::fromStdString(_uri) + "/" + identity;
  QString msg = result;
  _bw->publishText(uri, QString(), true, QList<RoutingObject *>(), ponum, msg,
                   QDateTime(), -1, "partial", false, false, [](QString err) {
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

void BWFrontEnd::onMessage(PMessage msg) {
  std::cerr << "DEBUG:: received message" << std::endl;
  QThread *workerThread = new QThread();
  BWWorker *worker = new BWWorker(msg, _onQuery, _numClients);
  worker->moveToThread(workerThread);
  connect(workerThread, &QThread::started, worker, &BWWorker::doWork);
  connect(worker, &BWWorker::doneWork, workerThread, &QThread::quit);
  connect(worker, &BWWorker::doneWork, worker, &BWWorker::deleteLater);
  connect(workerThread, &QThread::finished, workerThread,
          &QThread::deleteLater);
  connect(worker, &BWWorker::error, workerThread, &QThread::quit);
  connect(worker, &BWWorker::error, worker, &BWWorker::deleteLater);
  connect(worker, &BWWorker::error, this, &BWFrontEnd::workerReturnError);
  connect(worker, &BWWorker::doneWork, this, &BWFrontEnd::respond);
  workerThread->start();
  _numClients++;
}
