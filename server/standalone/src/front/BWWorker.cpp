#include "BWWorker.h"


BWWorker::BWWorker(PMessage message,
           Identification * id, 
           std::map<long, BWConnectionInfo *> *map,
           std::uniform_int_distribution<unsigned long long> *dis,
           std::mt19937 *gen,
           std::mutex *mutex,
           unsigned int *numClients 
           ) {
    _msg = message;
    _identification = id;
    _connInfoMap = map;
    _dis = dis;
    _gen = gen;
    _mutex = mutex;
    _numClients = numClients;

}
void BWWorker::doWork(){
  if(_msg->POs().length() != BW_MSG_LENGTH)
  {
    qDebug()<<"It's now a standard BW message\n";
    emit error();
  }
  BWConnectionInfo *connInfo = new BWConnectionInfo();
  assert(connInfo != nullptr);

  connInfo->session.reset(new Session());
  connInfo->session->id = (*_dis)(*_gen);
  connInfo->session->type = BOSSWAVE;
  connInfo->session->overallStart = getTime(); // log start of processing
  _mutex->lock();
  _connInfoMap->insert(std::make_pair(connInfo->session->id, connInfo));
  *_numClients = *_numClients + 1;
  _mutex->unlock();
  std::vector<const char*> contents;
  std::vector<int> lens;

  foreach(auto po, _msg->POs()) {
    contents.push_back(po->content());
    lens.push_back(po->length());
  }

  double fx, fy, cx, cy;
  int wdith, height;
  std::stringstream ss;
  for(int i = 5; i <= 8; i++) {
    ss<<std::string(contents[i], lens[i])<<" ";
  }
  ss>>fx>>fy>>cx>>cy;
  std::unique_ptr<cv::Mat> image(new cv::Mat());
  std::unique_ptr<CameraModel> camera(new CameraModel());
  std::unique_ptr<std::vector<char>> rawData;
  rawData.reset(new std::vector<char>());
  rawData->reserve(IMAGE_INIT_SIZE);
  rawData->insert(rawData->end(), contents[2], contents[2] + lens[2]);
  createData(*rawData, fx, fy, cx, cy, *image, *camera);
  if(image->empty()) {
    qDebug()<<"Creating image failed";
    emit error();
  }

  QCoreApplication::postEvent(_identification,
                              new QueryEvent(std::move(image),
                                             std::move(camera),
                                             std::move(connInfo->session)));
  connInfo->detected.acquire();
  std::string answer = "None";
  if (connInfo->names != nullptr && !connInfo->names->empty()) {
    answer = std::move(connInfo->names->at(0));
  }
  workComplete(connInfo);
  emit doneWork(QString::fromStdString(answer),QString::fromStdString(std::string(contents[1],lens[1])));


}
void BWWorker::createData(const std::vector<char> &data, double fx, double fy,
                                      double cx, double cy, cv::Mat &image,
                                      CameraModel &camera) {
  const bool copyData = false;
  image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);
  int width = image.cols;
  int height = image.rows;
  camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));
}
void BWWorker::workComplete(BWConnectionInfo *connInfo) {
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

  _mutex->lock();
  *_numClients -= 1;
  _connInfoMap->erase(session->id);
  _mutex->unlock();
  delete connInfo;
}


