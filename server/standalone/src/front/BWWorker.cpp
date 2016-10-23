#include "BWWorker.h"

/*void BWWorker::doWork(std::vector<const char*> *contents, std::vector<int> *lens){
  for(auto c : *contents) {
    qDebug()<<c;
  }
}*/

/*void BWWorker::doWork(){
  qDebug()<<"got you!\n"; 
}*/
BWWorker::BWWorker(PMessage message) {
  msg = message;
}
void BWWorker::doWork(){
   
  // qDebug() << "got message from " << message->getHeaderS("from");
   foreach(auto po, msg->POs()) {
     qDebug() << "PO"<<po->content() << " length " << po->length();
   }
   emit finished();
}
