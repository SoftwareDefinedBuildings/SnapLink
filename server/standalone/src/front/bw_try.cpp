class BWServer : public QObject
{
  friend class BWWorker;
public:
  BWServer();
  virtual ~BWServer(){};

  void startRun();

  void agentChanged();
  QByteArray mustGetEntity();

private:

  BW *_bw;
  QByteArray _entity;

}

BWServer::BWServer(){
  startRun();
}

void BWServer::startRun() {
  bw = BW::instance();
  QObject::connect(bw, &BW::agentChanged, &agentChanged);
  _entity = mustGetEntity();
  bw->connectAgent(entity);
  bw->setEntity(entity,[](QString err, QString vk){});
}

void BWServer::agentChanged() {
  bw->subscribe("scratch.ns/*",QString(),true,QList<RoutingObject*>(),QDateTime(),-1,QString(),false,false,[](PMessage msg){
           //Executed when a message arrives
           qDebug() << "got message from" << msg->getHeaderS("from");
           foreach(auto po, msg->POs()) {
               qDebug() << "PO"<< po->ponum()
                        << " length " << po->length();
               if(msg->getHeaderS("from") ==  QString("txa_ex9yFXVzzJkS0dtPafnRYX_DdkZ5-DQMMICXBzY=")) {
                   std::ofstream fout("output.png", std::ios::binary);
                   // fout<<"message is " << po->content();
                   fout.write(po->content(), po->length());
                   fout.close();
               }

           }
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
