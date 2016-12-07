#ifndef QTLIBBW_MESSAGE_H
#define QTLIBBW_MESSAGE_H

#include <QSharedPointer>
#include "agentconnection.h"

QT_FORWARD_DECLARE_CLASS(Message)
QT_FORWARD_DECLARE_CLASS(PayloadObject)


typedef QSharedPointer<Message> PMessage;
class Message
{
public:
    Message();
    static PMessage fromFrame(PFrame f);
    QString getHeaderS(QString key);
    QList<PayloadObject*> POs();
    QList<PayloadObject*> FilterPOs(int ponum);
    QList<PayloadObject*> FilterPOs(int ponum, int mask);

private:
    PFrame frame;
};

class PayloadObject
{
public:
    ~PayloadObject();
    static PayloadObject* load(int ponum, const char* dat, int length);
    int ponum();
    const char* content();
    QByteArray contentArray();
    int length();
protected:
    PayloadObject(int ponum, const char *data, int length) : m_ponum(ponum), m_data(data), m_length(length) {}
    int m_ponum;
    const char *m_data;
    int m_length;
};

PayloadObject* createBasePayloadObject(int ponum, QByteArray &contents);
PayloadObject* createBasePayloadObject(int ponum, const char* dat, int length);

#endif // QTLIBBW_MESSAGE_H
