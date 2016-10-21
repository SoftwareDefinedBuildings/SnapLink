#include <front/BWworker.h>

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


