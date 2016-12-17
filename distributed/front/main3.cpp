#include <QCoreApplication>
#include <allocations.h>
#include <libbw.h>

BW *bw;
QByteArray entity;
void agentChanged() {
  bw->setEntity(entity, [](QString err, QString vk) {
    // This is atrocious, I'm sorry. Sam is working on making overloads that
    // have better default params
    bw->subscribe("scratch.ns/*", QString(), true, QList<RoutingObject *>(),
                  QDateTime(), -1, QString(), false, false, [](PMessage msg) {
                    // Executed when a message arrives
                    qDebug() << "got message from " << msg->getHeaderS("from");
                    foreach (auto po, msg->POs()) {
                      qDebug() << "PO" << po->ponum() << " length "
                               << po->length();
                    }
                  });
  });
}

QByteArray mustGetEntity() {
  QString entitypath;
  // Try environment variable
  QByteArray a = qgetenv("BW2_DEFAULT_ENTITY");
  if (a.length() != 0) {
    entitypath = a.data();
  } else {
    // Try local file
    entitypath = "entity.ent";
  }
  QFile f(entitypath);
  if (!f.open(QIODevice::ReadOnly)) {
    qDebug() << "could not open entity file";
    return QByteArray();
  }
  QByteArray contents = f.readAll().mid(1);
  return contents;
}

int main(int argc, char *argv[]) {
  QCoreApplication a(argc, argv);
  bw = BW::instance();
  QObject::connect(bw, &BW::agentChanged, &agentChanged);

  entity = mustGetEntity();
  bw->connectAgent(entity);

  QTimer tmr;
  tmr.setSingleShot(false);
  tmr.setInterval(1000);
  QObject::connect(&tmr, &QTimer::timeout, [] {
    auto ponum = bwpo::num::Text;
    QString msg("hello world");
    bw->publishText("scratch.ns/kaifei/s.demo/ex/i.foo/signal/bar", QString(),
                    true, QList<RoutingObject *>(), ponum, msg, QDateTime(), -1,
                    "partial", false, false, [](QString err) {
                      if (!err.isEmpty()) {
                        qDebug() << "publish error: " << err;
                      } else {
                        qDebug() << "published ok";
                      }

                    });
  });
  tmr.start();

  return a.exec();
}
