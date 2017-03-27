#pragma once

#include "lib/front_end/FrontEnd.h"
#include <allocations.h>
#include <atomic>
#include <libbw.h>
#include <string>

class BWFrontEnd final : public QObject, public FrontEnd {
  Q_OBJECT

public:
  explicit BWFrontEnd(const std::string &uri);
  ~BWFrontEnd();

  bool start() final;
  void stop() final;

public slots:
  void run();
  void agentChanged(bool success, QString msg);
  void respond(QString result, QString identity);
  void error();

private:
  QByteArray getEntity();
  void onMessage(PMessage msg);

private:
  QThread _thread;
  BW &_bw;
  QByteArray _entity;
  std::string _uri;
  std::atomic<unsigned int> _numClients;
};
