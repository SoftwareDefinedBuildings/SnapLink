#include <iostream>
#include <libbw.h>
#include <allocations.h>
#include <sstream>
#include <string>
#include <fstream>
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

private:
  void parseMessage(PMessage msg);
  BW *_bw;
  QByteArray _entity;

};


