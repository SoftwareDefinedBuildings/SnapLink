#pragma once

#include <iostream>
#include <libbw.h>
#include <allocations.h>
#include <sstream>
#include <string>
#include <fstream>
#include "lib/front_end/bosswave/BWWorker.h"


#define MAX_CLIENTS 10
#define DEFAULT_CHANNEL "scratch.ns/cellmate"

class BWFrontEnd final : public QObject
{
  Q_OBJECT

public:
  explicit BWFrontEnd();
  ~BWFrontEnd();
  bool start(unsigned int maxClients);
  void stop();
  void registerOnQuery(std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> onQuery);
  void agentChanged();
  QByteArray mustGetEntity();

public slots:
  void publishResult(QString result, QString identity);
  void startRun();
  void workerReturnError();

signals:
  void signalBW();
  void askWorkerDoWork();


private:
  void parseMessage(PMessage msg);
  BW *_bw;
  QByteArray _entity;
  static const std::string none;
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> _onQuery;
  std::mutex _mutex;
  std::atomic<unsigned int> _maxClients;
  std::atomic<unsigned int> _numClients;
};
