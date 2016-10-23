#include "BWServer.h"
BWServer::BWServer(){
}

void BWServer::startRun() {
    qDebug()<<"starting run";
  _bw = BW::instance();
  QObject::connect(_bw, &BW::agentChanged,this, &BWServer::agentChanged);
  _entity = mustGetEntity();
  _bw->connectAgent(_entity);
  _bw->setEntity(_entity,[](QString err, QString vk){});
  
}
void BWServer::parseMessage(PMessage msg) {
  //remember to somehow free this thread and worker pointer later
  QThread *workerThread = new QThread();
  BWWorker *worker = new BWWorker(msg);
  worker->moveToThread(workerThread);
  QObject::connect(this,&BWServer::askWorkerDoWork,worker,&BWWorker::doWork);
  QObject::connect(worker, &BWWorker::finished, workerThread, &QThread::quit);
  QObject::connect(worker, &BWWorker::finished, worker, &BWWorker::deleteLater);
  QObject::connect(workerThread, &QThread::finished, workerThread, &QThread::deleteLater);
 // QObject::connect();
 // QObject::connect();
  workerThread->start();
 
  emit this->askWorkerDoWork(); 
}
void BWServer::agentChanged() {
  _bw->subscribe("scratch.ns/tongli",QString(),true,QList<RoutingObject*>(),QDateTime(),-1,QString(),false,false,[&](PMessage msg){this->parseMessage(msg);});
}

QByteArray BWServer::mustGetEntity() {
    QString entitypath;
    //Try environment variable
    QByteArray a = qgetenv("BW2_DEFAULT_ENTITY");
    if (a.length() != 0) {
        entitypath = a.data();
    } else {
        //Try local file
        entitypath = "entity.ent";
    }
    QFile f(entitypath);
    if (!f.open(QIODevice::ReadOnly))
    {
        qDebug() << "could not open entity file";
        return QByteArray();
    }
    QByteArray contents = f.readAll().mid(1);
    return contents;
}
