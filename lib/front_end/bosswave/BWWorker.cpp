#include "front_end/bosswave/BWWorker.h"

BWWorker::BWWorker(PMessage message,
                   std::function<std::vector<std::string>(
                       std::unique_ptr<cv::Mat> &&image,
                       std::unique_ptr<CameraModel> &&camera)> onQuery,
                  std::atomic<unsigned int>  *numClients) {
  _msg = message;
  _onQuery = onQuery;
  _numClients = numClients;
}
void BWWorker::doWork() {
  if (_msg->POs().length() != BW_MSG_LENGTH) {
    qDebug() << "It's now a standard BW message\n";
    emit error();
  }

  std::vector<const char *> contents;
  std::vector<int> lens;

  foreach (auto po, _msg->POs()) {
    contents.push_back(po->content());
    lens.push_back(po->length());
  }

  double fx, fy, cx, cy;
  int wdith, height;
  std::stringstream ss;
  // TODO: Add comments here to explain why 5 and 8
  for (int i = 5; i <= 8; i++) {
    ss << std::string(contents[i], lens[i]) << " ";
  }
  ss >> fx >> fy >> cx >> cy;
  std::unique_ptr<cv::Mat> image(new cv::Mat());
  std::unique_ptr<CameraModel> camera(new CameraModel());
  std::unique_ptr<std::vector<char>> rawData;
  rawData.reset(new std::vector<char>());
  rawData->reserve(IMAGE_INIT_SIZE);
  rawData->insert(rawData->end(), contents[2], contents[2] + lens[2]);
  createData(*rawData, fx, fy, cx, cy, *image, *camera);
  if (image->empty()) {
    qDebug() << "Creating image failed";
    emit error();
  }

  // QCoreApplication::postEvent(
  //     _identObj, new QueryEvent(std::move(image), std::move(camera),
  //                               std::move(connInfo->session)));
  // connInfo->detected.acquire();
  std::string answer = _onQuery(std::move(image), std::move(camera))[0];
  emit doneWork(QString::fromStdString(answer),
                QString::fromStdString(std::string(contents[1], lens[1])));
}
void BWWorker::createData(const std::vector<char> &data, double fx, double fy,
                          double cx, double cy, cv::Mat &image,
                          CameraModel &camera) {
  const bool copyData = false;
  image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

  // imwrite("image.jpg", image);

  int width = image.cols;
  int height = image.rows;
  camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));
}
