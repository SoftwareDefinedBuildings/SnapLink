#include "message.h"
#include "agentconnection.h"

Message::Message()
{

}

PMessage Message::fromFrame(PFrame f)
{
    Message *rv = new Message();
    rv->frame = f;
    return QSharedPointer<Message>(rv);
}

QString Message::getHeaderS(QString key)
{
    return frame->getHeaderS(key);
}

QList<PayloadObject*> Message::POs()
{
    return frame->pos;
}

QList<PayloadObject*> Message::FilterPOs(int ponum)
{
    auto rv = QList<PayloadObject*>();
    foreach(auto po, frame->pos)
    {
        if (po->ponum() == ponum)
        {
            rv.append(po);
        }
    }
    return rv;
}
QList<PayloadObject*> Message::FilterPOs(int ponum, int mask)
{
    auto rv = QList<PayloadObject*>();
    uint32_t realmask = (uint32_t)(0xFFFFFFFF << (32-mask));
    int fponum = ponum & realmask;
    foreach(auto po, frame->pos)
    {
        int mponum = po->ponum() & realmask;
        if (mponum == fponum)
        {
            rv.append(po);
        }
    }
    return rv;
}


PayloadObject::~PayloadObject()
{
    delete [] m_data;
}


// This will eventually construct subclasses too
PayloadObject* PayloadObject::load(int ponum, const char* dat, int size)
{
    return new PayloadObject(ponum, dat, size);
}


int PayloadObject::ponum()
{
    return m_ponum;
}
const char* PayloadObject::content()
{
    return m_data;
}
int PayloadObject::length()
{
    return m_length;
}

QByteArray PayloadObject::contentArray()
{
    return QByteArray::fromRawData(m_data, m_length);
}

PayloadObject* createBasePayloadObject(int ponum, QByteArray &contents)
{
    return createBasePayloadObject(ponum, contents.data(), contents.length());
}

PayloadObject* createBasePayloadObject(int ponum, const char* dat, int length)
{
    char* datcopy = new char [length];
    memcpy(datcopy,dat,length);
    return PayloadObject::load(ponum, datcopy, length);
}
