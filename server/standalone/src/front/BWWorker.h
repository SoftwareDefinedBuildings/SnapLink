#include <libbw.h>

class BWWorker : public QObject
{
  Q_OBJECT
public slots:
//void doWork(std::vector<const char*> *contents, std::vector<int> *lens);
//void doWork();
void doWork();
signals:
  void finished();

  public:
  BWWorker(PMessage message);

  private:
  PMessage msg;

};
