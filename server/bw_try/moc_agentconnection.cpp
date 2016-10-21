/****************************************************************************
** Meta object code from reading C++ file 'agentconnection.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../third_party/qtlibbw/agentconnection.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'agentconnection.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RoutingObject_t {
    QByteArrayData data[3];
    char stringdata0[29];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RoutingObject_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RoutingObject_t qt_meta_stringdata_RoutingObject = {
    {
QT_MOC_LITERAL(0, 0, 13), // "RoutingObject"
QT_MOC_LITERAL(1, 14, 8), // "isEntity"
QT_MOC_LITERAL(2, 23, 5) // "isDOT"

    },
    "RoutingObject\0isEntity\0isDOT"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RoutingObject[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       2,   14, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // properties: name, type, flags
       1, QMetaType::Bool, 0x00095003,
       2, QMetaType::Bool, 0x00095003,

       0        // eod
};

void RoutingObject::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{

#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty) {
        RoutingObject *_t = static_cast<RoutingObject *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< bool*>(_v) = _t->isEntity; break;
        case 1: *reinterpret_cast< bool*>(_v) = _t->isDOT; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        RoutingObject *_t = static_cast<RoutingObject *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->isEntity != *reinterpret_cast< bool*>(_v)) {
                _t->isEntity = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 1:
            if (_t->isDOT != *reinterpret_cast< bool*>(_v)) {
                _t->isDOT = *reinterpret_cast< bool*>(_v);
            }
            break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject RoutingObject::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RoutingObject.data,
      qt_meta_data_RoutingObject,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RoutingObject::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RoutingObject::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RoutingObject.stringdata0))
        return static_cast<void*>(const_cast< RoutingObject*>(this));
    return QObject::qt_metacast(_clname);
}

int RoutingObject::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    
#ifndef QT_NO_PROPERTIES
   if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 2;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
struct qt_meta_stringdata_Entity_t {
    QByteArrayData data[1];
    char stringdata0[7];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Entity_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Entity_t qt_meta_stringdata_Entity = {
    {
QT_MOC_LITERAL(0, 0, 6) // "Entity"

    },
    "Entity"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Entity[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void Entity::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject Entity::staticMetaObject = {
    { &RoutingObject::staticMetaObject, qt_meta_stringdata_Entity.data,
      qt_meta_data_Entity,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Entity::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Entity::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Entity.stringdata0))
        return static_cast<void*>(const_cast< Entity*>(this));
    return RoutingObject::qt_metacast(_clname);
}

int Entity::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = RoutingObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_AgentConnection_t {
    QByteArrayData data[15];
    char stringdata0[139];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AgentConnection_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AgentConnection_t qt_meta_stringdata_AgentConnection = {
    {
QT_MOC_LITERAL(0, 0, 15), // "AgentConnection"
QT_MOC_LITERAL(1, 16, 12), // "agentChanged"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 9), // "connected"
QT_MOC_LITERAL(4, 40, 3), // "msg"
QT_MOC_LITERAL(5, 44, 9), // "onConnect"
QT_MOC_LITERAL(6, 54, 7), // "onError"
QT_MOC_LITERAL(7, 62, 13), // "onArrivedData"
QT_MOC_LITERAL(8, 76, 8), // "initSock"
QT_MOC_LITERAL(9, 85, 10), // "doTransact"
QT_MOC_LITERAL(10, 96, 6), // "PFrame"
QT_MOC_LITERAL(11, 103, 1), // "f"
QT_MOC_LITERAL(12, 105, 11), // "onSslErrors"
QT_MOC_LITERAL(13, 117, 16), // "QList<QSslError>"
QT_MOC_LITERAL(14, 134, 4) // "errs"

    },
    "AgentConnection\0agentChanged\0\0connected\0"
    "msg\0onConnect\0onError\0onArrivedData\0"
    "initSock\0doTransact\0PFrame\0f\0onSslErrors\0"
    "QList<QSslError>\0errs"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AgentConnection[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   54,    2, 0x08 /* Private */,
       6,    0,   55,    2, 0x08 /* Private */,
       7,    0,   56,    2, 0x08 /* Private */,
       8,    0,   57,    2, 0x08 /* Private */,
       9,    1,   58,    2, 0x08 /* Private */,
      12,    1,   61,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 13,   14,

       0        // eod
};

void AgentConnection::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        AgentConnection *_t = static_cast<AgentConnection *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->agentChanged((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 1: _t->onConnect(); break;
        case 2: _t->onError(); break;
        case 3: _t->onArrivedData(); break;
        case 4: _t->initSock(); break;
        case 5: _t->doTransact((*reinterpret_cast< PFrame(*)>(_a[1]))); break;
        case 6: _t->onSslErrors((*reinterpret_cast< QList<QSslError>(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PFrame >(); break;
            }
            break;
        case 6:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QList<QSslError> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (AgentConnection::*_t)(bool , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&AgentConnection::agentChanged)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject AgentConnection::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_AgentConnection.data,
      qt_meta_data_AgentConnection,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *AgentConnection::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AgentConnection::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_AgentConnection.stringdata0))
        return static_cast<void*>(const_cast< AgentConnection*>(this));
    return QObject::qt_metacast(_clname);
}

int AgentConnection::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void AgentConnection::agentChanged(bool _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
