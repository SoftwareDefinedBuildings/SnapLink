#include "agentconnection.h"
#include "message.h"
#include "allocations.h"

#include <QCoreApplication>
#include <QtEndian>
#include <QTimer>
#include <QSslSocket>
#include <crypto.h>
#include <utils.h>

#include <functional>

Entity::Entity(int ronum, const char *data, int length, QObject* parent)
    : RoutingObject(ronum, data, length, parent)

{
    this->isEntity = true;

    QByteArray sk;
    if (ronum == bwpo::num::ROEntityWKey)
    {
        this->offset = 32;
        sk = QByteArray::fromRawData(data, this->offset);
        ronum = bwpo::num::ROEntity;
    }

    Q_ASSERT(ronum == bwpo::num::ROEntity);

    const char* content = this->content();
    this->vk = QByteArray::fromRawData(content, 32);

    int idx = 32;
    int ln;

    forever
    {
        switch (content[idx])
        {
        case 0x02: // Creation date
            Q_ASSERT_X(content[idx + 1] == 8, "Entity", "invalid creation date");
            idx += 2;
            this->created = qFromLittleEndian<qint64>((const unsigned char*) &content[idx]);
            idx += 8;
            break;
        case 0x03: // Expiry date
            Q_ASSERT_X(content[idx + 1] == 8, "Entity", "invalid expiry date");
            idx += 2;
            this->created = qFromLittleEndian<qint64>((const unsigned char*) &content[idx]);
            idx += 8;
            break;
        case 0x04: // Delegated revoker
            Q_ASSERT_X(content[idx + 1] == 32, "Entity", "invalid delegated revoker");
            idx += 2;
            this->revokers.append(QByteArray::fromRawData(&content[idx], 32));
            idx += 32;
            break;
        case 0x05: // Contact
            ln = (int) content[idx + 1];
            this->contact = QString(QByteArray::fromRawData(&content[idx + 2], ln));
            idx += (2 + ln);
            break;
        case 0x06: // Comment
            ln = (int) content[idx + 1];
            this->comment = QString(QByteArray::fromRawData(&content[idx + 2], ln));
            idx += (2 + ln);
            break;
        case 0x00: // End
            idx++;
            goto done;
        default:
            qWarning("Unknown Entity option type: %d", content[idx]);
            idx += (1 + (int) content[idx + 1]);
            break;
        }
    }

done:
    this->sig = QByteArray::fromRawData(&content[idx], 64);
    if (this->sk.isNull())
    {
        this->sk = qMove(sk);
    }
}

QByteArray Entity::getSigningBlob()
{
    QByteArray rv;

    if (this->sk.length() == 0 || this->length() == 0)
    {
        return rv;
    }

    rv.reserve(32 + this->length());
    rv.append(this->sk);
    rv.append(this->content(), this->length());
    return rv;
}


PFrame AgentConnection::newFrame(const char *type, quint32 seqno)
{
    if (seqno == 0)
        seqno = getSeqNo();
    auto rv = new Frame(this, type, seqno);
    return QSharedPointer<Frame>(rv);
}

Frame::~Frame()
{
    qDeleteAll(pos);
    qDeleteAll(headers);
    qDeleteAll(ros);
}

quint32 AgentConnection::getSeqNo()
{
    return (quint32)(seqno++);
}
void AgentConnection::onConnect()
{
    qDebug() << "socket connected";
    if (!m_ragent) {
        emit agentChanged(true, "");
    } //otherwise we do it later
}
void AgentConnection::onError()
{
    emit agentChanged(false, sock->errorString());
    qFatal("some kind of socket error or something...");
}
void AgentConnection::readRO(QStringList &tokens)
{
    //tokens0 is 'ro'
    Q_ASSERT(tokens.length() == 3);
    int ronum = tokens[1].toInt();
    int length = tokens[2].toInt();
    //For now do not support >16MB
    Q_ASSERT(length < 16*1024*1024);
    char *dat = new char[length];
    int readlen = sock->read(&dat[0],length);
    //We should be fully buffered
    Q_ASSERT(readlen == length);
    RoutingObject *ro = new RoutingObject(ronum, dat, length);
    curFrame->addRoutingObject(ro);
    char eatline[2];
    sock->read(&eatline[0],1);
}
void AgentConnection::readPO(QStringList &tokens)
{
    //tokens0 is 'po'
    Q_ASSERT(tokens.length() == 3);
    int ponum = tokens[1].split(':')[1].toInt();
    int length = tokens[2].toInt();
    //For now do not support >16MB
    Q_ASSERT(length < 16*1024*1024);
    char *dat = new char[length];
    int readlen = sock->read(&dat[0],length);
    //We should be fully buffered
    Q_ASSERT(readlen == length);
    PayloadObject *ro = PayloadObject::load(ponum, dat, length);
    curFrame->addPayloadObject(ro);
    char eatline[2];
    sock->read(&eatline[0],1);
}
void AgentConnection::readKV(QStringList &tokens)
{
    //tokens0 is 'kv'
    Q_ASSERT(tokens.length() == 3);
    QString key = tokens[1];
    int length = tokens[2].toInt();
    //For now do not support >16MB
    Q_ASSERT(length < 16*1024*1024);
    char *dat = new char[length];
    int readlen = sock->read(&dat[0],length);
    //We should be fully buffered
    Q_ASSERT(readlen == length);
    Header *h = new Header(key,dat, length);
    curFrame->addHeader(h);
    char eatline[2];
    sock->read(&eatline[0],1);
}
void AgentConnection::onArrivedData()
{
    if (m_ragent && m_ragent_handshake==0)
    {
        if (sock->bytesAvailable() < 128)
            return;
        char hs[128];
        qint64 l = sock->read(hs,128);
        Q_ASSERT(l==128);
        //This is RVK:SIG:NONCE
        QSslCertificate peercert = ((QSslSocket*)sock)->peerCertificate();
        if (peercert.isNull()) {
            qFatal("no peer certificate");
        }
        QByteArray der = peercert.toDer();
        QByteArray rvk = QByteArray(&hs[0],32);
        QByteArray rsig = QByteArray(&hs[32],64);
        bool okay = VerifyBlob(rvk,rsig,der);
        if (!okay) {
            qFatal("Remote signature was invalid");
        }
        QByteArray nonce = QByteArray(&hs[96],32);
        QByteArray oursig = QByteArray(64,0);
        qDebug() <<"our sk" << m_our_sk.toBase64(QByteArray::Base64UrlEncoding);
        qDebug() <<"our vk" << m_our_vk.toBase64(QByteArray::Base64UrlEncoding);
        SignBlob(m_our_sk, m_our_vk,&nonce,&oursig);
        bool ok = VerifyBlob(m_our_vk,oursig,nonce);
        qDebug() << "we think sig is " << ok;
        qDebug()<< "nonce was" << nonce.toBase64();
        int done = sock->write(m_our_vk);
        Q_ASSERT(done == 32);
        done = sock->write(oursig);
        qDebug() << "our sig was" << oursig.toBase64();
        qDebug() <<"our sk2" << m_our_sk.toBase64(QByteArray::Base64UrlEncoding);
        qDebug() <<"our vk2" << m_our_vk.toBase64(QByteArray::Base64UrlEncoding);
        Q_ASSERT(done == 64);
        m_ragent_handshake=1;
    }
    if (m_ragent && m_ragent_handshake==1)
    {
        qDebug() << "stage 2";
        if (sock->bytesAvailable()<4)
            return;
        char st [4];
        qint64 l = sock->read(st,4);
        Q_ASSERT(l==4);
        if (st[0] != 'O') {
            qFatal("remote dislikes us");
        }
        m_ragent_handshake=2;
        emit agentChanged(true, "");
        qDebug() << "finished remote agent handshake";
    }
    if (curFrame.isNull())
    {
        //New frame, read the header
        if (sock->bytesAvailable() < 27)
            return; //Wait until we have the full header
        char hdr[28];
        qint64 l = sock->read(hdr,27);
        Q_ASSERT(l==27);
        //Remember header is
        //    4          15         26
        //CMMD 10DIGITLEN 10DIGITSEQ\n
        hdr[4]=0;
        hdr[15]=0;
        hdr[26]=0;//Kill the \n too
        int length = QString(&hdr[5]).toInt(); //not currently used
        int seq = QString(&hdr[16]).toInt();
        curFrame = newFrame(&hdr[0], seq);
        waitingFor = length;
    }
    //We have a frame, we are loading parts of it
    if (sock->bytesAvailable() < waitingFor)
        return; //wait until it is fully loaded

    //All of this we can now do without blocking as teh data is buffered
    char linebuf[256];
    while(1) {
        auto linelen = sock->readLine(linebuf, 256);
        Q_ASSERT(linelen < 255);
        Q_ASSERT(linelen > 0);
        linebuf[linelen-1] = 0; //kill the newline
        QString line(linebuf);
        QStringList tokens = line.split(' ');
        if (tokens[0] == "kv") {
            readKV(tokens);
        }else if (tokens[0] == "po") {
            readPO(tokens);
        }else if (tokens[0] == "ro") {
            readRO(tokens);
        }else if (tokens[0] == "end") {
            //This frame is finished
            auto nf = curFrame;
            curFrame.reset();
            invokeOnThread(QCoreApplication::instance()->thread(),
                           std::function<void (PFrame)>([this](PFrame f)
                           {
                               this->onArrivedFrame(f);
                           }), nf);
            QMetaObject::invokeMethod(this,"onArrivedData",Qt::QueuedConnection);
            return;
        }
    }
}

void AgentConnection::initSock()
{
    if (!m_ragent)
    {
        sock = new QTcpSocket(this);
        connect(sock, &QTcpSocket::connected, this, &AgentConnection::onConnect);
        connect(sock,static_cast<void(QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
                this, &AgentConnection::onError);
        connect(sock, &QTcpSocket::readyRead, this, &AgentConnection::onArrivedData);
        sock->connectToHost(m_desthost, m_destport);
    }
    else
    {
        QSslSocket *secsock = new QSslSocket(this);
        sock = secsock;
        connect(sock, &QTcpSocket::connected, this, &AgentConnection::onConnect);
        connect(sock,static_cast<void(QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
                this, &AgentConnection::onError);
        connect(secsock, SIGNAL(sslErrors(QList<QSslError>)),
                        this, SLOT(onSslErrors(QList<QSslError>)));
        connect(sock, &QTcpSocket::readyRead, this, &AgentConnection::onArrivedData);
        secsock->connectToHostEncrypted(m_desthost, m_destport);
    }
}

void AgentConnection::onSslErrors(QList<QSslError> errs)
{
    bool okay = true;
    foreach (auto err, errs)
    {
        if (err.error() == QSslError::HostNameMismatch ||
            err.error() == QSslError::SelfSignedCertificate)
        {
            continue;
        }
        okay = false;
    }
    if (okay) {
        ((QSslSocket*)sock)->ignoreSslErrors(errs);
    } else {
        qFatal("unexpected ssl errors");
    }
}

void AgentConnection::onArrivedFrame(PFrame f)
{
    Q_ASSERT(QThread::currentThread() == QCoreApplication::instance()->thread());

    //We need to determine which transaction this belongs to, and forward the frame there.
    if (f->isType(Frame::HELLO))
    {
        Q_ASSERT(!have_received_helo);
        have_received_helo = true;
        return;
    }

    Q_ASSERT(outstanding.contains(f->seqno()));
    bool present;
    bool final = f->getHeaderBool("finished", &present);
    Q_ASSERT_X(present, "frame decode", "finished kv missing");
    outstanding.value(f->seqno())(f, final);
    if (final) {
        outstanding.remove(f->seqno());
    }
}
void AgentConnection::transact(PFrame f, function<void (PFrame, bool)> cb)
{
    Q_ASSERT(QThread::currentThread() == QCoreApplication::instance()->thread());

    outstanding[f->seqno()] = cb;

    //We want to move this to a different thread if we are not on the agent's thread
    QMetaObject::invokeMethod(this, "doTransact", Q_ARG(PFrame, f));
}

void AgentConnection::transact(QObject *to, PFrame f, function<void (PFrame, bool)> cb)
{
    //Basically the callback to transact will wrap the real callback, and move it
    //to the thread that 'to' lives in (probably the GUI thread).
    auto safecb = [=](PFrame f, bool fin)
    {
        invokeOnThread(to->thread(), cb, f, fin);
    };

    invokeOnThread(QCoreApplication::instance()->thread(),
                   std::function<void()>([=]()
                   {
                       this->transact(f, safecb);
                   }));
}

void AgentConnection::doTransact(PFrame f)
{
    Q_ASSERT(QThread::currentThread() == this->m_thread);
    //Now that we know we are on the right thread, there is no need to lock on the socket access
    f->writeTo(sock);
}
void Frame::writeTo(QIODevice *o)
{
    //We omit frame length, BW server can handle that
    QString hdr = QString("%1 %2 %3\n").arg(m_type,4).arg(0,10,10,QChar('0')).arg(m_seqno,10,10,QChar('0'));
    o->write(hdr.toLatin1());
    foreach(auto kv, headers)
    {
        QString hdr = QString("kv %1 %2\n").arg(kv->key()).arg(kv->length());
        o->write(hdr.toLatin1());
        o->write(kv->content(), kv->length());
        o->write("\n",1);
    }
    foreach(auto ro, ros)
    {
        QString hdr = QString("ro %1 %2\n").arg(ro->ronum()).arg(ro->length());
        o->write(hdr.toLatin1());
        o->write(ro->content(), ro->length());
        o->write("\n",1);
    }
    foreach(auto po, pos)
    {
        QString hdr = QString("po :%1 %2\n").arg(po->ponum()).arg(po->length());
        o->write(hdr.toLatin1());
        o->write(po->content(),po->length());
        o->write("\n",1);
    }
    o->write("end\n",4);
}

//Returns false if not there
bool Frame::getHeaderBool(QString key, bool *valid)
{
    foreach(auto h, headers)
    {
        if (h->key() == key)
        {
            if (valid != NULL)
                *valid = true;
            return h->asBool();
        }
    }
    if (valid != NULL)
        *valid = false;
    return false;
}

QByteArray Frame::getHeaderB(QString key, bool* valid)
{
    foreach(auto h, headers)
    {
        if (h->key() == key)
        {
            if (valid != NULL)
                *valid = true;
            return h->asByteArray();
        }
    }
    if (valid != NULL)
        *valid = false;
    return QByteArray();
}

//Returns "" if not there
QString Frame::getHeaderS(QString key, bool *valid)
{
    foreach(auto h, headers)
    {
        if (h->key() == key)
        {
            if (valid != NULL)
                *valid = true;
            return h->asString();
        }
    }
    if (valid != NULL)
        *valid = false;
    return QString("");
}

//Returns -1 if not there
int Frame::getHeaderI(QString key, bool *valid)
{
    foreach(auto h, headers)
    {
        if (h->key() == key)
        {
            if (valid != NULL)
                *valid = true;
            return h->asInt();
        }
    }
    if (valid != NULL)
        *valid = false;
    return -1;
}

bool Header::asBool()
{
    if (asString().toLower() == "true")
    {
        return true;
    }
    return false;
}
int Header::asInt()
{
    return asString().toInt();
}

QString Header::asString()
{
    return QString::fromUtf8(m_data, m_length);
}

QByteArray Header::asByteArray()
{
    return QByteArray(m_data, m_length);
}
