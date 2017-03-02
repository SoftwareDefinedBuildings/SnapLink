#pragma once

#include <iostream>
#include <libbw.h>
#include <allocations.h>
#include <string>
#include <fstream>
#include <memory>
#include "lib/data/CameraModel.h"
#include <vector>
#include <atomic>

class BWFrontEnd final : public QObject
{
  Q_OBJECT

public:
  explicit BWFrontEnd();
  ~BWFrontEnd();

  // start front end thread asynchronously
  void start(const std::string &uri);
  // stop front end thread synchronously
  void stop();

  void registerOnQuery(std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> onQuery);


public slots:
  void run();
  void agentChanged(bool success, QString msg);
  void respond(QString result, QString identity);
  void error();

signals:
  void signalBW();

private:
  QByteArray getEntity();
  void onMessage(PMessage msg);

private:
  std::unique_ptr<QThread> _thread;
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> _onQuery;
  std::shared_ptr<BW> _bw;
  QByteArray _entity;
  std::string _uri;
  std::atomic<unsigned int> _numClients;
};
