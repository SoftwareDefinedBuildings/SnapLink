#include <front/BW.h>
/*
void BWWorker::doWork(const PMessage &msg, BWServer *server) { 
  if(msg->POs().length() != BW_MSG_LENGTH)
  {
     std::cout<<"It's not a standard BW input\n";
     return;
  }
  ConnectionInfo *connInfo = new ConnectionInfo();
  assert(connInfo != nullptr);

  connInfo->session.reset(new Session());
  connInfo->session->id = server->_dis(server->_gen);
  connInfo->session->type = BOSSWAVE;

  server->_mutex.lock();
  server->_connInfoMap.insert(
      std::make_pair(connInfo->session->id, connInfo));
  server->setNumClients(server->getNumClients() + 1);
  server->_mutex.unlock();

  std::vector<const char*> contents;
  std::vector<int> lens;

  foreach(auto po, msg->POs()) {
    contents.push_back(po->content());
    lens.push_back(po->length());
  }
  double fx, fy, cx, cy;
  int wdith, height;
  std::stringstream ss;
  for(int i = 5; i <= 8; i++) {
    ss<<contents[i]<<" ";
  }
  ss>>fx>>fy>>cx>>cy;
  std::unique_ptr<cv::Mat> image(new cv::Mat());
  std::unique_ptr<CameraModel> camera(new CameraModel());
  std::unique_ptr<std::vector<char>> rawData;
  rawData.reset(new std::vector<char>());
  rawData->reserve(IMAGE_INIT_SIZE);
  rawData->insert(rawData->end(), contents[2], contents[2] + lens[2]);
  server->createData(*rawData, fx, fy, cx, cy, *image, *camera); 
  if(!image->empty()) {
    std::cout<<"Creating image failed\n";
    return;
  }
  QCoreApplication::postEvent((QObject*)server->_featureStage,
                              new QueryEvent(std::move(image),
                                             std::move(camera),
                                             std::move(connInfo->session)));
  connInfo->detected.acquire();
  std::string answer = "None";
  if (connInfo->names != nullptr && !connInfo->names->empty()) {
    answer = std::move(connInfo->names->at(0));
  }
  workComplete(server, connInfo);
  emit doneWork(answer, std::string(contents[1],lens[1]));
}

void BWWorker::workComplete(BWServer *server, ConnectionInfo *connInfo) {
  std::unique_ptr<Session> session = std::move(connInfo->session);
  if (session != nullptr) {
    session->overallEnd = getTime(); // log processing end time

    std::cout << "TAG_TIME overall "
              << session->overallEnd - session->overallStart << " ms"
              << std::endl;
    std::cout << "TAG_TIME features "
              << session->featuresEnd - session->featuresStart << " ms"
              << std::endl;
    std::cout << "TAG_TIME words " << session->wordsEnd - session->wordsStart
              << " ms" << std::endl;
    std::cout << "TAG_TIME perspective "
              << session->perspectiveEnd - session->perspectiveStart << " ms"
              << std::endl;
  }

  server->_mutex.lock();
  server->setNumClients(server->getNumClients() - 1);
  server->_connInfoMap.erase(session->id);
  server->_mutex.unlock();
  delete connInfo;
}
*/
BWServer::BWServer()
{
    _numClients = 0;
    _bw = BW::instance();
    _featureStage = nullptr;
    _maxClients = MAX_CLIENTS;
    _gen = std::mt19937(std::random_device()());
    startRun();
}  

BWServer::~BWServer() {
  _bw = nullptr;
  _numClients = 0;
  _featureStage = nullptr;
}

void BWServer::startRun(){
    //QCoreApplication a(argc, argv);
    QObject::connect(_bw, SIGNAL(BW::agentChanged(bool connected, QString msg)),this, SLOT(BWServer::agentChanged()));
    _entity = mustGetEntity();
    _bw->connectAgent(_entity);
    _bw->setEntity(_entity,[](QString err, QString vk){});
    //return a.exec();
}

int BWServer::getMaxClients() const { return _maxClients; }

int BWServer::getNumClients() const { return _numClients; }

void BWServer::setNumClients(int numClients) { _numClients = numClients; }

void BWServer::setFeatureStage(FeatureStage *featureStage) {
  _featureStage = featureStage;
}

void BWServer::publishResult(std::string result, std::string identity) {
  auto ponum = bwpo::num::Text;
  QString msg = QString::fromStdString(result);
  std::string channel = DEFAULT_CHANNEL;
  _bw->publishText(QString::fromStdString(channel) + "/" + QString::fromStdString(identity),QString(),true,QList<RoutingObject*>(),ponum,msg,QDateTime(),-1,"partial",false,false,[](QString err) {
      if (!err.isEmpty()) {
          qDebug() << "publish error: " << err;
      } else {
          qDebug() << "published ok";
      }
  });
}

bool BWServer::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();
    // find() const is thread-safe
    const auto iter = _connInfoMap.find(session->id);
    ConnectionInfo *connInfo = iter->second;
    connInfo->names = detectionEvent->takeNames();
    connInfo->session = std::move(session);
    connInfo->detected.release();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::unique_ptr<Session> session = failureEvent->takeSession();
    // find() const is thread-safe
    const auto iter = _connInfoMap.find(session->id);
    ConnectionInfo *connInfo = iter->second;
    connInfo->session = std::move(session);
    connInfo->detected.release();
    return true;
  }
  return QObject::event(event);
}

void BWServer::agentChanged()
{
   // _bw->subscribe(DEFAULT_CHANNEL,QString(),true,QList<RoutingObject*>(),QDateTime(),-1,QString(),false,false,BWServer::parseMessage);
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

void BWServer::parseMessage(PMessage msg) {
    QThread workerThread;
    BWWorker *worker = new BWWorker();
    worker->moveToThread(&workerThread);
    QObject::connect(&workerThread, &QThread::finished, worker, &QObject::deleteLater);
    QObject::connect(this, SIGNAL(BWServer::askWorkerDoWork(const PMessage &msg, BWServer *server)), worker, SLOT(BWWorker::doWork(const PMessage &msg, BWServer *server)));
    QObject::connect(worker, SIGNAL(BWWorker::doneWork(std::string result, std::string identity)), this, SLOT(BWServer::publishResult(std::string result, std::string identity)));
    workerThread.start();
    emit askWorkerDoWork(msg, this);
}


void BWServer::createData(const std::vector<char> &data, double fx, double fy,
                            double cx, double cy, cv::Mat &image,
                            CameraModel &camera) {
  // no data copy is needed because conn info
  const bool copyData = false;
  image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

  int width = image.cols;
  int height = image.rows;
  camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));
}
