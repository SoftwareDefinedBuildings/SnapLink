#include "lib/front_end/bosswave/BWFrontEnd.h"
#include "lib/front_end/bosswave/BWWorker.h"

BWFrontEnd::BWFrontEnd(const std::string &uri)
    : _bw(BW::instance()), _uri(uri), _numClients(0) {}

BWFrontEnd::~BWFrontEnd() {
  stop();
  _bw.reset();
  _numClients = 0;
}

bool BWFrontEnd::start() {
  _thread.reset(new QThread());
  this->moveToThread(_thread.get());
  connect(_thread.get(), &QThread::started, this, &BWFrontEnd::run);
  _thread->start();

  return true;
}

void BWFrontEnd::stop() {
  _thread->exit();
  _thread.reset();
}

void BWFrontEnd::run() {
  connect(_bw.get(), &BW::agentChanged, this, &BWFrontEnd::agentChanged);
  _entity = getEntity();
  _bw->connectAgent(_entity);
  _bw->setEntity(_entity, [](QString, QString) {});
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
                       std::cerr
                           << "publish error: " << err.toUtf8().constData()
                           << std::endl;
                     } else {
                       std::cerr << "published ok" << std::endl;
                     }
                   });
  _numClients--;
}

void BWFrontEnd::error() { _numClients--; }

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
    std::cerr << "could not open entity file" << std::endl;
    return QByteArray();
  }
  QByteArray contents = f.readAll().mid(1);
  return contents;
}

void BWFrontEnd::onMessage(PMessage msg) {
  std::cerr << "DEBUG:: received message" << std::endl;
  // workerThread memory will be freed using QThread::deleteLater
  QThread *workerThread = new QThread();
  // worker memory will be freed using BWWorker::deleteLater
  BWWorker *worker = new BWWorker(msg, getOnQuery(), _numClients);

  worker->moveToThread(workerThread);

  connect(workerThread, &QThread::started, worker, &BWWorker::process);
  connect(worker, &BWWorker::done, workerThread, &QThread::quit);
  connect(worker, &BWWorker::error, workerThread, &QThread::quit);
  connect(workerThread, &QThread::finished, worker, &BWWorker::deleteLater);
  connect(workerThread, &QThread::finished, workerThread,
          &QThread::deleteLater);

  connect(worker, &BWWorker::done, this, &BWFrontEnd::respond);
  connect(worker, &BWWorker::error, this, &BWFrontEnd::error);

  workerThread->start();
  _numClients++;
}
