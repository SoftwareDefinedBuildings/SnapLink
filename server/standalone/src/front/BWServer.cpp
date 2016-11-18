#include "BWServer.h"
BWServer::BWServer(){ 
    _numClients = 0;
    _identification = nullptr; 
    _gen = std::mt19937(std::random_device()());
    _bw = BW::instance();
}

BWServer::~BWServer() {
  _numClients = 0;
  _identification = nullptr;
}
int BWServer::getMaxClients() const { return _maxClients; }
int BWServer::getNumClients() const { return _numClients; }
void BWServer::setNumClients(int numClients) { _numClients = numClients; }
void BWServer::setIdentification(Identification *identification) {
  _identification = identification;
}
void BWServer::startRun() {
  qDebug()<<"starting run";
  _maxClients = MAX_CLIENTS;
  QObject::connect(_bw, &BW::agentChanged,this, &BWServer::agentChanged);
  _entity = mustGetEntity();
  _bw->connectAgent(_entity);
  _bw->setEntity(_entity,[](QString err, QString vk){});
}
void BWServer::parseMessage(PMessage msg) {
  QThread *workerThread = new QThread();
  BWWorker *worker = new BWWorker(msg, _identification, &_connInfoMap, &_dis, &_gen, &_mutex, &_numClients);
  worker->moveToThread(workerThread);
  QObject::connect(this,&BWServer::askWorkerDoWork,worker,&BWWorker::doWork);
  QObject::connect(worker, &BWWorker::doneWork, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::doneWork, worker, &BWWorker::deleteLater);
  QObject::connect(workerThread, &QThread::finished, workerThread, &QThread::deleteLater);
  QObject::connect(worker, &BWWorker::error, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::error, worker, &BWWorker::deleteLater);
  QObject::connect(worker, &BWWorker::error, this, &BWServer::workerReturnError);
  QObject::connect(worker, &BWWorker::doneWork, this, &BWServer::publishResult);
  workerThread->start();
  setNumClients(getNumClients() + 1); 
  emit this->askWorkerDoWork(); 
}
void BWServer::agentChanged() {
  
  _bw->subscribe(DEFAULT_CHANNEL,QString(),true,QList<RoutingObject*>(),QDateTime(),-1,QString(),false,false,[&](PMessage msg){this->parseMessage(msg);});
}

QByteArray BWServer::mustGetEntity() {
    QString entitypath;
    //Try environment variable
    QByteArray a = qgetenv("BW2_DEFAULT_ENTITY");
    if (a.length() != 0) {
        entitypath = a.data();
    } else {
        //Try local file
        entitypath = "entity.ent";
    }
    QFile f(entitypath);
    if (!f.open(QIODevice::ReadOnly))
    {
        qDebug() << "could not open entity file";
        return QByteArray();
    }
    QByteArray contents = f.readAll().mid(1);
    return contents;
}
void BWServer::publishResult(QString result, QString identity) {
  auto ponum = bwpo::num::Text;
  QString msg = result;
  std::string channel = DEFAULT_CHANNEL;
  _bw->publishText(QString::fromStdString(channel) + "/" + identity ,QString(),true,QList<RoutingObject*>(),ponum,msg,QDateTime(),-1,"partial",false,false,[](QString err) {
      if (!err.isEmpty()) {
          qDebug() << "publish error: " << err;
      } else {
          qDebug() << "published ok";
      }
  });
  setNumClients(getNumClients() - 1);
}


bool BWServer::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();
    // find() const is thread-safe
    const auto iter = _connInfoMap.find(session->id);
    BWConnectionInfo *connInfo = iter->second;
    connInfo->names = detectionEvent->takeNames();
    connInfo->session = std::move(session);
    connInfo->detected.release();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::unique_ptr<Session> session = failureEvent->takeSession();
    // find() const is thread-safe
    const auto iter = _connInfoMap.find(session->id);
    BWConnectionInfo *connInfo = iter->second;
    connInfo->session = std::move(session);
    connInfo->detected.release();
    return true;
  }
  return QObject::event(event);
}

void BWServer::workerReturnError() {
  //publish error message to identity channel
  setNumClients(getNumClients() - 1);
}
