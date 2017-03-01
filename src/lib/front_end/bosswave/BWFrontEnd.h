#pragma once

#include <iostream>
#include <libbw.h>
#include <allocations.h>
#include <sstream>
#include <string>
#include <fstream>
#include "lib/front_end/bosswave/BWWorker.h"

#define DEFAULT_CHANNEL "scratch.ns/cellmate"

class BWFrontEnd final : public QObject
{
  Q_OBJECT

public:
  explicit BWFrontEnd();
  ~BWFrontEnd();

  // start front end thread asynchronously
  void start(unsigned int maxClients);
  // stop front end thread synchronously
  void stop();

  void registerOnQuery(std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> onQuery);


public slots:
  void run();
  void agentChanged(bool success, QString msg);
  void publishResult(QString result, QString identity);
  void workerReturnError();

signals:
  void signalBW();
  void askWorkerDoWork();

private:
  QByteArray getEntity();
  void parseMessage(PMessage msg);

private:
  static const std::string none;
  std::unique_ptr<QThread> _thread;
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> _onQuery;
  std::shared_ptr<BW> _bw;
  QByteArray _entity;
  std::mutex _mutex;
  std::atomic<unsigned int> _maxClients;
  std::atomic<unsigned int> _numClients;
};
