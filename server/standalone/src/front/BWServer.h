#include <iostream>
#include <libbw.h>
#include <allocations.h>
#include <sstream>
#include <string>
#include <fstream>
#include <front/BWWorker.h>
class BWServer : public QObject
{
  Q_OBJECT
public:
  BWServer();
  virtual ~BWServer(){};
  void agentChanged();
  QByteArray mustGetEntity();
  public slots:
  void startRun();
signals:
  void signalBW();
  //void askWorkerDoWork();
  //void askWorkerDoWork(std::vector<const char*> *contents, std::vector<int> *lens);
  void askWorkerDoWork();
  
private:
  void parseMessage(PMessage msg);
  BW *_bw;
  QByteArray _entity;

};


