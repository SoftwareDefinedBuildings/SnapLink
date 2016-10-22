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




}
void BWServer::agentChanged() {
  _bw->subscribe("scratch.ns/tongli",QString(),true,QList<RoutingObject*>(),QDateTime(),-1,QString(),false,false,[](PMessage msg){
           //Executed when a message arrives
           qDebug() << "got message from" << msg->getHeaderS("from");
           foreach(auto po, msg->POs()) {
               qDebug() << "PO"<< po->ponum()
                        << " length " << po->length(); 
               qDebug() << "contents" << po->content();
           }
           std::ofstream fout("/root/workspace/bw2python/examples/output.txt");
           fout<<"got me!\n";
           fout.close();
       });
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
