#pragma once

#include <iostream>
#include <libbw.h>
#include <allocations.h>
#include <sstream>
#include <string>
#include <fstream>
#include "front_end/bosswave/BWWorker.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"

#define MAX_CLIENTS 10
#define DEFAULT_CHANNEL "scratch.ns/cellmate"

class BWServer : public QObject
{
  Q_OBJECT

public:
  BWServer();
  virtual ~BWServer();
  void agentChanged();
  QByteArray mustGetEntity();
  
public slots:
  void publishResult(QString result, QString identity);
  void startRun();
  void workerReturnError();

signals:
  void signalBW();
  void askWorkerDoWork();

public:
  int getMaxClients() const;
  int getNumClients() const;
  void setNumClients(int numClients);
  void setIdentificationObj(IdentificationObj *identObj);

protected:
  virtual bool event(QEvent *event);

private:
  void parseMessage(PMessage msg);
  BW *_bw;
  QByteArray _entity;
  unsigned int _maxClients;
  unsigned int _numClients;
  std::mutex _mutex;
  std::mt19937 _gen;
  std::uniform_int_distribution<unsigned long long> _dis;
  std::map<long, BWConnectionInfo *> _connInfoMap;
  IdentificationObj *_identObj;
};

