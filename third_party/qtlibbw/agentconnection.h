#ifndef QTLIBBW_AGENTCONNECTION_H
#define QTLIBBW_AGENTCONNECTION_H

#include <QCoreApplication>
#include <QObject>
#include <QDebug>
#include <QSharedPointer>
#include <QTcpSocket>
#include <QThread>
#include <string>
#include <functional>
#include <QJSValue>
#include <QJSEngine>
#include <QSslError>
using std::function;

QT_FORWARD_DECLARE_CLASS(PayloadObject)
QT_FORWARD_DECLARE_CLASS(Message)
QT_FORWARD_DECLARE_CLASS(AgentConnection)

class RoutingObject : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isEntity MEMBER isEntity)
    Q_PROPERTY(bool isDOT MEMBER isDOT)

public:
    RoutingObject(QObject* parent = nullptr)
        : QObject(parent), isEntity(false), isDOT(false), offset(0), m_ronum(0),
          m_data(nullptr), m_length(0) {}
    RoutingObject(int ronum, const char *data, int length, QObject* parent = nullptr)
        : QObject(parent), isEntity(false), isDOT(false), offset(0), m_ronum(ronum),
          m_data(data), m_length(length) {}
    ~RoutingObject()
    {
        delete[] m_data;
    }
    int ronum()
    {
        return m_ronum;
    }

    const char* content()
    {
        return m_data + offset;
    }

    int length()
    {
        return m_length - offset;
    }

    bool isEntity;
    bool isDOT;

protected:
    int offset;

private:
    int m_ronum;
    const char* m_data;
    int m_length;
};

class Entity : public RoutingObject
{
    Q_OBJECT

public:
    Entity(QObject* parent = nullptr) : RoutingObject(parent) {};
    Entity(int ronum, const char* data, int length, QObject* parent = nullptr);

    QByteArray getSigningBlob();
    QByteArray vk;
    QByteArray sk;
private:
    QByteArray sig;
    int64_t expires;
    int64_t created;
    QList<QByteArray> revokers;
    QString contact;
    QString comment;
};

class Header
{
public:
    Header(QString key, char *data, int length) : m_key(key), m_data(data), m_length(length) {}
    Header(QString key, QString val)
    {
        m_key = key;

        QByteArray utf8 = val.toUtf8();
        m_data = new char[utf8.length()];
        m_length = utf8.length();
        memcpy(m_data,utf8.data(), m_length);
    }
    ~Header()
    {
        delete [] m_data;
    }
    const QString& key()
    {
        return m_key;
    }
    const char* content ()
    {
        return m_data;
    }
    int length() {
        return m_length;
    }
    bool asBool();
    int asInt();
    QString asString();
    QByteArray asByteArray();
private:
    QString m_key;
    char* m_data;
    int m_length;
};

Q_DECLARE_METATYPE(RoutingObject*)

/*
class Status
{
public:
    Status(bool iserror, QString msg)
        : m_msg(msg), m_iserror(iserror)
    {

    }

    bool isError()
    {
        return m_iserror;
    }
    static Status okay()
    {
        return Status(false,"");
    }
    QString msg()
    {
        return m_msg;
    }

    static Status error(QString msg)
    {
        return Status(true, msg);
    }
private:
    QString m_msg;
    bool m_iserror;
};
*/
/*
template <typename ...Tz>
using Res = std::function<void(Tz...)>;
*/



template <typename ...Tz>
class Res
{
public:
    Res(): javascript(false)
    {
        wrap = [](Tz...){};
    }
    Res(std::function<void(Tz...)> cb): javascript(false)
    {
        wrap = cb;
    }
    template <typename F>
    Res(F f) : Res(std::function<void(Tz...)>(f)) {}
    Res(QJSEngine* e, QJSValue callback): javascript(true)
    {
        if (!callback.isCallable())
        {
            qFatal("Trying to construct Res with non function JS Value");
        }
        wrap = [=](Tz... args) mutable
        {
            QJSValueList l;
            convertE(e, l, args...);
            callback.call(l);
        };
    }
    Res(const Res& other)
    {
        if (other.javascript)
        {
            Q_ASSERT(QThread::currentThread() == QCoreApplication::instance()->thread());
        }
        this->wrap = other.wrap;
        this->javascript = other.javascript;
    }

    void operator() (Tz ...args) const
    {
        wrap(args...);
    }
private:
    std::function<void(Tz...)> wrap;
    bool javascript;
};

class Frame
{
public:
    constexpr static const char* HELLO             = "helo";
    constexpr static const char* PUBLISH           = "publ";
    constexpr static const char* SUBSCRIBE         = "subs";
    constexpr static const char* PERSIST           = "pers";
    constexpr static const char* LIST              = "list";
    constexpr static const char* QUERY             = "quer";
    constexpr static const char* TAP_SUBSCRIBE     = "tsub";
    constexpr static const char* TAP_QUERY         = "tque";
    constexpr static const char* MAKE_DOT          = "makd";
    constexpr static const char* MAKE_ENTITY       = "make";
    constexpr static const char* MAKE_CHAIN        = "makc";
    constexpr static const char* BUILD_CHAIN       = "bldc";
    constexpr static const char* SET_ENTITY        = "sete";
    constexpr static const char* PUT_DOT           = "putd";
    constexpr static const char* PUT_ENTITY        = "pute";
    constexpr static const char* PUT_CHAIN         = "putc";
    constexpr static const char* ENTITY_BALANCE    = "ebal";
    constexpr static const char* ADDRESS_BALANCE   = "abal";
    constexpr static const char* BC_PARAMS         = "bcip";
    constexpr static const char* TRANSFER          = "xfer";
    constexpr static const char* MK_SHORT_ALIAS    = "mksa";
    constexpr static const char* MK_LONG_ALIAS     = "mkla";
    constexpr static const char* RESOLVE_ALIAS     = "resa";
    constexpr static const char* NEW_DRO           = "ndro";
    constexpr static const char* ACCEPT_DRO        = "adro";
    constexpr static const char* RESOLVE_REGISTRY  = "rsro";
    constexpr static const char* UPDATE_SRV        = "usrv";
    constexpr static const char* LIST_DRO          = "ldro";
    constexpr static const char* REVOKE_DRO        = "rdro";
    constexpr static const char* REVOKE_DRO_ACCEPT = "rdra";
    constexpr static const char* REVOKE_RO         = "revk";
    constexpr static const char* PUT_REVOCATION    = "prvk";

    constexpr static const char* MAKE_VIEW         = "mkvw";
    constexpr static const char* SUBSCRIBE_VIEW    = "vsub";
    constexpr static const char* PUBLISH_VIEW      = "vpub";
    constexpr static const char* LIST_VIEW         = "vlst";
    constexpr static const char* UNSUBSCRIBE       = "usub";

    constexpr static const char* RESPONSE = "resp";
    constexpr static const char* RESULT   = "rslt";

    Frame(AgentConnection *agent, const char* type, quint32 seqno)
        :agent(agent), m_seqno(seqno)
    {
        strncpy(&m_type[0],type,4);
        m_type[4] = 0;
        pos = QList<PayloadObject*>();
        headers = QList<Header*>();
        ros = QList<RoutingObject*>();
    }
    ~Frame();

    quint32 seqno()
    {
        return m_seqno;
    }

    bool isType(const char* type)
    {
        return strcmp(m_type, type) == 0;
    }
    const char* type()
    {
        return &m_type[0];
    }

    //Returns false if not there
    bool getHeaderBool(QString key, bool *valid = nullptr);
    //Returns empty array if not there
    QByteArray getHeaderB(QString key, bool* valid = nullptr);
    //Returns "" if not there
    QString getHeaderS(QString key, bool *valid = nullptr);
    //Returns -1 if not there
    int getHeaderI(QString key, bool *valid = nullptr);

    template <typename ...Tz> bool checkResponse(Res<QString,Tz...> cb, Tz ...args)
    {
        Q_ASSERT(isType(RESPONSE));
        bool ok;
        auto status = getHeaderS("status", &ok);
        Q_ASSERT(ok);
        if (status == "okay")
        {
            return true;
        }
        cb(getHeaderS("reason"), args...);
        return false;
    }

    void addPayloadObject(PayloadObject *po)
    {
        pos.append(po);
    }

    QList<PayloadObject*> getPayloadObjects()
    {
        return pos;
    }

    QList<RoutingObject*> getRoutingObjects()
    {
        return ros;
    }

    void addHeader(Header *h)
    {
        headers.append(h);
    }
    void addHeader(QString key, QString val)
    {
        Header* h = new Header(key, val);
        addHeader(h);
    }
    void addHeaderB(QString key, QByteArray a)
    {
        char* dat = new char[a.size()];
        memcpy(&dat[0],&a.data()[0], a.size());
        addHeader(new Header(key,dat,a.size()));
    }

    void addRoutingObject(RoutingObject* ro)
    {
        ros.append(ro);
    }
    void writeTo(QIODevice *o);
private:
    AgentConnection *agent;
    char m_type[5];
    const quint32 m_seqno;
    QList<PayloadObject*> pos;
    QList<RoutingObject*> ros;
    QList<Header*> headers;

    friend Message;
};

typedef QSharedPointer<Frame> PFrame;

Q_DECLARE_METATYPE(PFrame)
Q_DECLARE_METATYPE(function<void(PFrame,bool)>)

class AgentConnection : public QObject
{
    Q_OBJECT
public:
    explicit AgentConnection(QObject *parent = 0)
        : QObject(parent), m_ragent(false), m_our_sk(), m_our_vk(), m_ragent_handshake(0)
    {
        qRegisterMetaType<PFrame>();
        qRegisterMetaType<function<void(PFrame,bool)>>();
        seqno = 1;
        have_received_helo = false;
        outstanding = QHash<quint32, function<void(PFrame,bool)>>();
        //All our stuff will happen on this thread
        m_thread = new QThread(this);
        m_thread->start();
       // m_desthost = target;
       // m_destport = port;
        moveToThread(m_thread);

     /*
        sock = new QTcpSocket(this);
        connect(sock, &QTcpSocket::connected, this, &AgentConnection::onConnect);
        connect(sock,static_cast<void(QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
                this, &AgentConnection::onError);
        connect(sock, &QTcpSocket::readyRead, this, &AgentConnection::onArrivedData);
        sock->connectToHost(target, port);*/
    }

//    bool waitForConnection()
//    {
//        //TODO apparently this does not work well on windows
//        return sock->waitForConnected();
//    }
    void beginConnection(QString target, qint16 port)
    {
        m_desthost = target;
        m_destport = port;

        //We might be on another thread.
        QMetaObject::invokeMethod(this,"initSock");
    }
    void beginRagentConnection(QByteArray our_sk, QByteArray our_vk, QString target, qint16 port, QByteArray remote_vk)
    {
        m_desthost = target;
        m_destport = port;
        m_ragent = true;
        m_remote_vk = remote_vk;
        m_our_sk = our_sk;
        m_our_vk = our_vk;
        //We might be on another thread.
        QMetaObject::invokeMethod(this,"initSock");
    }

    void transact(QObject *to, PFrame f, function<void(PFrame f, bool final)> cb);
    PFrame newFrame(const char *type, quint32 seqno=0);
private:
    quint32 getSeqNo();
    void readRO(QStringList &tokens);
    void readPO(QStringList &tokens);
    void readKV(QStringList &tokens);
    QAtomicInt seqno;
    QTcpSocket *sock;
    QThread    *m_thread;
    PFrame  curFrame;
    int waitingFor;
    QHash<quint32, function<void(PFrame f, bool final)>> outstanding;
    void transact(PFrame f, function<void(PFrame f, bool final)> cb);
    void onArrivedFrame(PFrame f);
    bool have_received_helo;
    QString m_desthost;
    qint16 m_destport;
    bool m_ragent;
    QByteArray m_our_sk;
    QByteArray m_our_vk;
    QByteArray m_remote_vk;
    int m_ragent_handshake;
private slots:
    void onConnect();
    void onError();
    void onArrivedData();
    void initSock();
    void doTransact(PFrame f);
    void onSslErrors(QList<QSslError> errs);
signals:
    void agentChanged(bool connected, QString msg);

};

#endif // QTLIBBW_AGENTCONNECTION_H
