/****************************************************************************
** Meta object code from reading C++ file 'bosswave.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../third_party/qtlibbw/bosswave.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'bosswave.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_BW_t {
    QByteArrayData data[92];
    char stringdata0[1102];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BW_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BW_t qt_meta_stringdata_BW = {
    {
QT_MOC_LITERAL(0, 0, 2), // "BW"
QT_MOC_LITERAL(1, 3, 12), // "agentChanged"
QT_MOC_LITERAL(2, 16, 0), // ""
QT_MOC_LITERAL(3, 17, 7), // "success"
QT_MOC_LITERAL(4, 25, 3), // "msg"
QT_MOC_LITERAL(5, 29, 6), // "fromDF"
QT_MOC_LITERAL(6, 36, 2), // "df"
QT_MOC_LITERAL(7, 39, 12), // "createEntity"
QT_MOC_LITERAL(8, 52, 6), // "params"
QT_MOC_LITERAL(9, 59, 8), // "QJSValue"
QT_MOC_LITERAL(10, 68, 7), // "on_done"
QT_MOC_LITERAL(11, 76, 9), // "createDOT"
QT_MOC_LITERAL(12, 86, 14), // "createDOTChain"
QT_MOC_LITERAL(13, 101, 14), // "publishMsgPack"
QT_MOC_LITERAL(14, 116, 11), // "publishText"
QT_MOC_LITERAL(15, 128, 16), // "subscribeMsgPack"
QT_MOC_LITERAL(16, 145, 6), // "on_msg"
QT_MOC_LITERAL(17, 152, 13), // "subscribeText"
QT_MOC_LITERAL(18, 166, 11), // "unsubscribe"
QT_MOC_LITERAL(19, 178, 6), // "handle"
QT_MOC_LITERAL(20, 185, 13), // "setEntityFile"
QT_MOC_LITERAL(21, 199, 8), // "filename"
QT_MOC_LITERAL(22, 208, 9), // "setEntity"
QT_MOC_LITERAL(23, 218, 7), // "keyfile"
QT_MOC_LITERAL(24, 226, 20), // "setEntityFromEnviron"
QT_MOC_LITERAL(25, 247, 10), // "buildChain"
QT_MOC_LITERAL(26, 258, 13), // "buildAnyChain"
QT_MOC_LITERAL(27, 272, 12), // "queryMsgPack"
QT_MOC_LITERAL(28, 285, 9), // "on_result"
QT_MOC_LITERAL(29, 295, 9), // "queryText"
QT_MOC_LITERAL(30, 305, 4), // "list"
QT_MOC_LITERAL(31, 310, 17), // "publishDOTWithAcc"
QT_MOC_LITERAL(32, 328, 4), // "blob"
QT_MOC_LITERAL(33, 333, 7), // "account"
QT_MOC_LITERAL(34, 341, 10), // "publishDOT"
QT_MOC_LITERAL(35, 352, 20), // "publishEntityWithAcc"
QT_MOC_LITERAL(36, 373, 13), // "publishEntity"
QT_MOC_LITERAL(37, 387, 11), // "setMetadata"
QT_MOC_LITERAL(38, 399, 3), // "uri"
QT_MOC_LITERAL(39, 403, 3), // "key"
QT_MOC_LITERAL(40, 407, 3), // "val"
QT_MOC_LITERAL(41, 411, 11), // "delMetadata"
QT_MOC_LITERAL(42, 423, 11), // "getMetadata"
QT_MOC_LITERAL(43, 435, 14), // "getMetadataKey"
QT_MOC_LITERAL(44, 450, 19), // "publishChainWithAcc"
QT_MOC_LITERAL(45, 470, 12), // "publishChain"
QT_MOC_LITERAL(46, 483, 14), // "unresolveAlias"
QT_MOC_LITERAL(47, 498, 16), // "resolveLongAlias"
QT_MOC_LITERAL(48, 515, 2), // "al"
QT_MOC_LITERAL(49, 518, 17), // "resolveShortAlias"
QT_MOC_LITERAL(50, 536, 20), // "resolveEmbeddedAlias"
QT_MOC_LITERAL(51, 557, 15), // "resolveRegistry"
QT_MOC_LITERAL(52, 573, 14), // "entityBalances"
QT_MOC_LITERAL(53, 588, 14), // "addressBalance"
QT_MOC_LITERAL(54, 603, 4), // "addr"
QT_MOC_LITERAL(55, 608, 22), // "getBCInteractionParams"
QT_MOC_LITERAL(56, 631, 22), // "setBCInteractionParams"
QT_MOC_LITERAL(57, 654, 13), // "confirmations"
QT_MOC_LITERAL(58, 668, 7), // "timeout"
QT_MOC_LITERAL(59, 676, 6), // "maxAge"
QT_MOC_LITERAL(60, 683, 13), // "transferEther"
QT_MOC_LITERAL(61, 697, 4), // "from"
QT_MOC_LITERAL(62, 702, 2), // "to"
QT_MOC_LITERAL(63, 705, 5), // "ether"
QT_MOC_LITERAL(64, 711, 24), // "newDesignatedRouterOffer"
QT_MOC_LITERAL(65, 736, 4), // "nsvk"
QT_MOC_LITERAL(66, 741, 7), // "Entity*"
QT_MOC_LITERAL(67, 749, 2), // "dr"
QT_MOC_LITERAL(68, 752, 27), // "revokeDesignatedRouterOffer"
QT_MOC_LITERAL(69, 780, 39), // "revokeAcceptanceOfDesignatedR..."
QT_MOC_LITERAL(70, 820, 4), // "drvk"
QT_MOC_LITERAL(71, 825, 2), // "ns"
QT_MOC_LITERAL(72, 828, 12), // "revokeEntity"
QT_MOC_LITERAL(73, 841, 2), // "vk"
QT_MOC_LITERAL(74, 844, 9), // "revokeDOT"
QT_MOC_LITERAL(75, 854, 4), // "hash"
QT_MOC_LITERAL(76, 859, 17), // "publishRevocation"
QT_MOC_LITERAL(77, 877, 25), // "getDesignatedRouterOffers"
QT_MOC_LITERAL(78, 903, 27), // "acceptDesignatedRouterOffer"
QT_MOC_LITERAL(79, 931, 28), // "setDesignatedRouterSRVRecord"
QT_MOC_LITERAL(80, 960, 3), // "srv"
QT_MOC_LITERAL(81, 964, 15), // "createLongAlias"
QT_MOC_LITERAL(82, 980, 5), // "getVK"
QT_MOC_LITERAL(83, 986, 10), // "createView"
QT_MOC_LITERAL(84, 997, 5), // "query"
QT_MOC_LITERAL(85, 1003, 20), // "Res<QString,BWView*>"
QT_MOC_LITERAL(86, 1024, 16), // "RegistryValidity"
QT_MOC_LITERAL(87, 1041, 12), // "StateUnknown"
QT_MOC_LITERAL(88, 1054, 10), // "StateValid"
QT_MOC_LITERAL(89, 1065, 12), // "StateExpired"
QT_MOC_LITERAL(90, 1078, 12), // "StateRevoked"
QT_MOC_LITERAL(91, 1091, 10) // "StateError"

    },
    "BW\0agentChanged\0\0success\0msg\0fromDF\0"
    "df\0createEntity\0params\0QJSValue\0on_done\0"
    "createDOT\0createDOTChain\0publishMsgPack\0"
    "publishText\0subscribeMsgPack\0on_msg\0"
    "subscribeText\0unsubscribe\0handle\0"
    "setEntityFile\0filename\0setEntity\0"
    "keyfile\0setEntityFromEnviron\0buildChain\0"
    "buildAnyChain\0queryMsgPack\0on_result\0"
    "queryText\0list\0publishDOTWithAcc\0blob\0"
    "account\0publishDOT\0publishEntityWithAcc\0"
    "publishEntity\0setMetadata\0uri\0key\0val\0"
    "delMetadata\0getMetadata\0getMetadataKey\0"
    "publishChainWithAcc\0publishChain\0"
    "unresolveAlias\0resolveLongAlias\0al\0"
    "resolveShortAlias\0resolveEmbeddedAlias\0"
    "resolveRegistry\0entityBalances\0"
    "addressBalance\0addr\0getBCInteractionParams\0"
    "setBCInteractionParams\0confirmations\0"
    "timeout\0maxAge\0transferEther\0from\0to\0"
    "ether\0newDesignatedRouterOffer\0nsvk\0"
    "Entity*\0dr\0revokeDesignatedRouterOffer\0"
    "revokeAcceptanceOfDesignatedRouterOffer\0"
    "drvk\0ns\0revokeEntity\0vk\0revokeDOT\0"
    "hash\0publishRevocation\0getDesignatedRouterOffers\0"
    "acceptDesignatedRouterOffer\0"
    "setDesignatedRouterSRVRecord\0srv\0"
    "createLongAlias\0getVK\0createView\0query\0"
    "Res<QString,BWView*>\0RegistryValidity\0"
    "StateUnknown\0StateValid\0StateExpired\0"
    "StateRevoked\0StateError"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BW[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      51,   14, // methods
       0,    0, // properties
       1,  564, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,  269,    2, 0x06 /* Public */,

 // methods: name, argc, parameters, tag, flags
       5,    1,  274,    2, 0x02 /* Public */,
       7,    2,  277,    2, 0x02 /* Public */,
      11,    2,  282,    2, 0x02 /* Public */,
      12,    2,  287,    2, 0x02 /* Public */,
      13,    2,  292,    2, 0x02 /* Public */,
      14,    2,  297,    2, 0x02 /* Public */,
      15,    3,  302,    2, 0x02 /* Public */,
      17,    3,  309,    2, 0x02 /* Public */,
      18,    2,  316,    2, 0x02 /* Public */,
      20,    2,  321,    2, 0x02 /* Public */,
      22,    2,  326,    2, 0x02 /* Public */,
      24,    1,  331,    2, 0x02 /* Public */,
      25,    2,  334,    2, 0x02 /* Public */,
      26,    2,  339,    2, 0x02 /* Public */,
      27,    2,  344,    2, 0x02 /* Public */,
      29,    2,  349,    2, 0x02 /* Public */,
      30,    2,  354,    2, 0x02 /* Public */,
      31,    3,  359,    2, 0x02 /* Public */,
      34,    2,  366,    2, 0x02 /* Public */,
      35,    3,  371,    2, 0x02 /* Public */,
      36,    2,  378,    2, 0x02 /* Public */,
      37,    4,  383,    2, 0x02 /* Public */,
      41,    3,  392,    2, 0x02 /* Public */,
      42,    2,  399,    2, 0x02 /* Public */,
      43,    3,  404,    2, 0x02 /* Public */,
      44,    3,  411,    2, 0x02 /* Public */,
      45,    2,  418,    2, 0x02 /* Public */,
      46,    2,  423,    2, 0x02 /* Public */,
      47,    2,  428,    2, 0x02 /* Public */,
      49,    2,  433,    2, 0x02 /* Public */,
      50,    2,  438,    2, 0x02 /* Public */,
      51,    2,  443,    2, 0x02 /* Public */,
      52,    1,  448,    2, 0x02 /* Public */,
      53,    2,  451,    2, 0x02 /* Public */,
      55,    1,  456,    2, 0x02 /* Public */,
      56,    4,  459,    2, 0x02 /* Public */,
      60,    4,  468,    2, 0x02 /* Public */,
      64,    4,  477,    2, 0x02 /* Public */,
      68,    4,  486,    2, 0x02 /* Public */,
      69,    4,  495,    2, 0x02 /* Public */,
      72,    2,  504,    2, 0x02 /* Public */,
      74,    2,  509,    2, 0x02 /* Public */,
      76,    3,  514,    2, 0x02 /* Public */,
      77,    2,  521,    2, 0x02 /* Public */,
      78,    4,  526,    2, 0x02 /* Public */,
      79,    4,  535,    2, 0x02 /* Public */,
      81,    4,  544,    2, 0x02 /* Public */,
      82,    0,  553,    2, 0x02 /* Public */,
      83,    2,  554,    2, 0x02 /* Public */,
      83,    2,  559,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    3,    4,

 // methods: parameters
    QMetaType::Int, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9, 0x80000000 | 9,    8,   16,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9, 0x80000000 | 9,    8,   16,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   19,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   21,   10,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 9,   23,   10,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   28,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   28,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,    8,   28,
    QMetaType::Void, QMetaType::QByteArray, QMetaType::Int, 0x80000000 | 9,   32,   33,   10,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 9,   32,   10,
    QMetaType::Void, QMetaType::QByteArray, QMetaType::Int, 0x80000000 | 9,   32,   33,   10,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 9,   32,   10,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QString, 0x80000000 | 9,   38,   39,   40,   10,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, 0x80000000 | 9,   38,   39,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   38,   10,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, 0x80000000 | 9,   38,   39,   10,
    QMetaType::Void, QMetaType::QByteArray, QMetaType::Int, 0x80000000 | 9,   32,   33,   10,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 9,   32,   10,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 9,   32,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   48,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   48,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   48,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   39,   10,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   54,   10,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::QReal, QMetaType::QReal, QMetaType::QReal, 0x80000000 | 9,   57,   58,   59,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, QMetaType::Double, 0x80000000 | 9,   61,   62,   63,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, 0x80000000 | 66, 0x80000000 | 9,   33,   65,   67,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, 0x80000000 | 66, 0x80000000 | 9,   33,   65,   67,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, 0x80000000 | 66, 0x80000000 | 9,   33,   70,   71,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   73,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   75,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QByteArray, 0x80000000 | 9,   33,   32,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 9,   65,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, 0x80000000 | 66, 0x80000000 | 9,   33,   70,   71,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, 0x80000000 | 66, 0x80000000 | 9,   33,   80,   67,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::QByteArray, QMetaType::QByteArray, 0x80000000 | 9,   33,   39,   40,   10,
    QMetaType::QString,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 85,   84,   10,
    QMetaType::Void, QMetaType::QVariantMap, 0x80000000 | 9,   84,   10,

 // enums: name, flags, count, data
      86, 0x0,    5,  568,

 // enum data: key, value
      87, uint(BW::StateUnknown),
      88, uint(BW::StateValid),
      89, uint(BW::StateExpired),
      90, uint(BW::StateRevoked),
      91, uint(BW::StateError),

       0        // eod
};

void BW::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BW *_t = static_cast<BW *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->agentChanged((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 1: { int _r = _t->fromDF((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 2: _t->createEntity((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 3: _t->createDOT((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 4: _t->createDOTChain((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 5: _t->publishMsgPack((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 6: _t->publishText((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 7: _t->subscribeMsgPack((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 8: _t->subscribeText((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 9: _t->unsubscribe((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 10: _t->setEntityFile((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 11: _t->setEntity((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 12: _t->setEntityFromEnviron((*reinterpret_cast< QJSValue(*)>(_a[1]))); break;
        case 13: _t->buildChain((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 14: _t->buildAnyChain((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 15: _t->queryMsgPack((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 16: _t->queryText((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 17: _t->list((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 18: _t->publishDOTWithAcc((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 19: _t->publishDOT((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 20: _t->publishEntityWithAcc((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 21: _t->publishEntity((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 22: _t->setMetadata((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 23: _t->delMetadata((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 24: _t->getMetadata((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 25: _t->getMetadataKey((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 26: _t->publishChainWithAcc((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 27: _t->publishChain((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 28: _t->unresolveAlias((*reinterpret_cast< QByteArray(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 29: _t->resolveLongAlias((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 30: _t->resolveShortAlias((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 31: _t->resolveEmbeddedAlias((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 32: _t->resolveRegistry((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 33: _t->entityBalances((*reinterpret_cast< QJSValue(*)>(_a[1]))); break;
        case 34: _t->addressBalance((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 35: _t->getBCInteractionParams((*reinterpret_cast< QJSValue(*)>(_a[1]))); break;
        case 36: _t->setBCInteractionParams((*reinterpret_cast< qreal(*)>(_a[1])),(*reinterpret_cast< qreal(*)>(_a[2])),(*reinterpret_cast< qreal(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 37: _t->transferEther((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 38: _t->newDesignatedRouterOffer((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< Entity*(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 39: _t->revokeDesignatedRouterOffer((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< Entity*(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 40: _t->revokeAcceptanceOfDesignatedRouterOffer((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< Entity*(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 41: _t->revokeEntity((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 42: _t->revokeDOT((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 43: _t->publishRevocation((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QByteArray(*)>(_a[2])),(*reinterpret_cast< QJSValue(*)>(_a[3]))); break;
        case 44: _t->getDesignatedRouterOffers((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        case 45: _t->acceptDesignatedRouterOffer((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< Entity*(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 46: _t->setDesignatedRouterSRVRecord((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< Entity*(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 47: _t->createLongAlias((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QByteArray(*)>(_a[2])),(*reinterpret_cast< QByteArray(*)>(_a[3])),(*reinterpret_cast< QJSValue(*)>(_a[4]))); break;
        case 48: { QString _r = _t->getVK();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        case 49: _t->createView((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< Res<QString,BWView*>(*)>(_a[2]))); break;
        case 50: _t->createView((*reinterpret_cast< QVariantMap(*)>(_a[1])),(*reinterpret_cast< QJSValue(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 6:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 7:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 8:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 9:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 10:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 11:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 12:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 13:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 14:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 15:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 16:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 17:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 18:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 19:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 20:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 21:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 22:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 23:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 24:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 25:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 26:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 27:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 28:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 29:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 30:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 31:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 32:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 33:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 34:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 35:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 36:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 37:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 38:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Entity* >(); break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 39:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Entity* >(); break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 40:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Entity* >(); break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 41:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 42:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 43:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 44:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 45:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Entity* >(); break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 46:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 2:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Entity* >(); break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 47:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 3:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        case 50:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QJSValue >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (BW::*_t)(bool , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BW::agentChanged)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject BW::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BW.data,
      qt_meta_data_BW,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BW::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BW::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BW.stringdata0))
        return static_cast<void*>(const_cast< BW*>(this));
    return QObject::qt_metacast(_clname);
}

int BW::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 51)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 51;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 51)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 51;
    }
    return _id;
}

// SIGNAL 0
void BW::agentChanged(bool _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_MetadataTupleJS_t {
    QByteArrayData data[3];
    char stringdata0[27];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MetadataTupleJS_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MetadataTupleJS_t qt_meta_stringdata_MetadataTupleJS = {
    {
QT_MOC_LITERAL(0, 0, 15), // "MetadataTupleJS"
QT_MOC_LITERAL(1, 16, 5), // "value"
QT_MOC_LITERAL(2, 22, 4) // "time"

    },
    "MetadataTupleJS\0value\0time"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MetadataTupleJS[] = {

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
       1, QMetaType::QString, 0x00095003,
       2, QMetaType::QDateTime, 0x00095003,

       0        // eod
};

void MetadataTupleJS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{

#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty) {
        MetadataTupleJS *_t = static_cast<MetadataTupleJS *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->value; break;
        case 1: *reinterpret_cast< QDateTime*>(_v) = _t->time; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        MetadataTupleJS *_t = static_cast<MetadataTupleJS *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->value != *reinterpret_cast< QString*>(_v)) {
                _t->value = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 1:
            if (_t->time != *reinterpret_cast< QDateTime*>(_v)) {
                _t->time = *reinterpret_cast< QDateTime*>(_v);
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

const QMetaObject MetadataTupleJS::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MetadataTupleJS.data,
      qt_meta_data_MetadataTupleJS,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MetadataTupleJS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MetadataTupleJS::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MetadataTupleJS.stringdata0))
        return static_cast<void*>(const_cast< MetadataTupleJS*>(this));
    return QObject::qt_metacast(_clname);
}

int MetadataTupleJS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
struct qt_meta_stringdata_BlockChainInteractionParams_t {
    QByteArrayData data[9];
    char stringdata0[110];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BlockChainInteractionParams_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BlockChainInteractionParams_t qt_meta_stringdata_BlockChainInteractionParams = {
    {
QT_MOC_LITERAL(0, 0, 27), // "BlockChainInteractionParams"
QT_MOC_LITERAL(1, 28, 12), // "confirmation"
QT_MOC_LITERAL(2, 41, 7), // "timeout"
QT_MOC_LITERAL(3, 49, 6), // "maxAge"
QT_MOC_LITERAL(4, 56, 10), // "currentAge"
QT_MOC_LITERAL(5, 67, 12), // "currentBlock"
QT_MOC_LITERAL(6, 80, 5), // "peers"
QT_MOC_LITERAL(7, 86, 12), // "highestBlock"
QT_MOC_LITERAL(8, 99, 10) // "difficulty"

    },
    "BlockChainInteractionParams\0confirmation\0"
    "timeout\0maxAge\0currentAge\0currentBlock\0"
    "peers\0highestBlock\0difficulty"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BlockChainInteractionParams[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       8,   14, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // properties: name, type, flags
       1, QMetaType::QReal, 0x00095003,
       2, QMetaType::QReal, 0x00095003,
       3, QMetaType::QReal, 0x00095003,
       4, QMetaType::QReal, 0x00095003,
       5, QMetaType::QReal, 0x00095003,
       6, QMetaType::QReal, 0x00095003,
       7, QMetaType::QReal, 0x00095003,
       8, QMetaType::QReal, 0x00095003,

       0        // eod
};

void BlockChainInteractionParams::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{

#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty) {
        BlockChainInteractionParams *_t = static_cast<BlockChainInteractionParams *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< qreal*>(_v) = _t->confirmations; break;
        case 1: *reinterpret_cast< qreal*>(_v) = _t->timeout; break;
        case 2: *reinterpret_cast< qreal*>(_v) = _t->maxAge; break;
        case 3: *reinterpret_cast< qreal*>(_v) = _t->currentAge; break;
        case 4: *reinterpret_cast< qreal*>(_v) = _t->currentBlock; break;
        case 5: *reinterpret_cast< qreal*>(_v) = _t->peers; break;
        case 6: *reinterpret_cast< qreal*>(_v) = _t->highestBlock; break;
        case 7: *reinterpret_cast< qreal*>(_v) = _t->difficulty; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        BlockChainInteractionParams *_t = static_cast<BlockChainInteractionParams *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->confirmations != *reinterpret_cast< qreal*>(_v)) {
                _t->confirmations = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 1:
            if (_t->timeout != *reinterpret_cast< qreal*>(_v)) {
                _t->timeout = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 2:
            if (_t->maxAge != *reinterpret_cast< qreal*>(_v)) {
                _t->maxAge = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 3:
            if (_t->currentAge != *reinterpret_cast< qreal*>(_v)) {
                _t->currentAge = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 4:
            if (_t->currentBlock != *reinterpret_cast< qreal*>(_v)) {
                _t->currentBlock = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 5:
            if (_t->peers != *reinterpret_cast< qreal*>(_v)) {
                _t->peers = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 6:
            if (_t->highestBlock != *reinterpret_cast< qreal*>(_v)) {
                _t->highestBlock = *reinterpret_cast< qreal*>(_v);
            }
            break;
        case 7:
            if (_t->difficulty != *reinterpret_cast< qreal*>(_v)) {
                _t->difficulty = *reinterpret_cast< qreal*>(_v);
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

const QMetaObject BlockChainInteractionParams::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BlockChainInteractionParams.data,
      qt_meta_data_BlockChainInteractionParams,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BlockChainInteractionParams::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BlockChainInteractionParams::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BlockChainInteractionParams.stringdata0))
        return static_cast<void*>(const_cast< BlockChainInteractionParams*>(this));
    return QObject::qt_metacast(_clname);
}

int BlockChainInteractionParams::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    
#ifndef QT_NO_PROPERTIES
   if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 8;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 8;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 8;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 8;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 8;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
struct qt_meta_stringdata_BalanceInfo_t {
    QByteArrayData data[5];
    char stringdata0[37];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BalanceInfo_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BalanceInfo_t qt_meta_stringdata_BalanceInfo = {
    {
QT_MOC_LITERAL(0, 0, 11), // "BalanceInfo"
QT_MOC_LITERAL(1, 12, 4), // "addr"
QT_MOC_LITERAL(2, 17, 5), // "human"
QT_MOC_LITERAL(3, 23, 7), // "decimal"
QT_MOC_LITERAL(4, 31, 5) // "value"

    },
    "BalanceInfo\0addr\0human\0decimal\0value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BalanceInfo[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       4,   14, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // properties: name, type, flags
       1, QMetaType::QString, 0x00095003,
       2, QMetaType::QString, 0x00095003,
       3, QMetaType::QString, 0x00095003,
       4, QMetaType::QReal, 0x00095003,

       0        // eod
};

void BalanceInfo::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{

#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty) {
        BalanceInfo *_t = static_cast<BalanceInfo *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->addr; break;
        case 1: *reinterpret_cast< QString*>(_v) = _t->human; break;
        case 2: *reinterpret_cast< QString*>(_v) = _t->decimal; break;
        case 3: *reinterpret_cast< qreal*>(_v) = _t->value; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        BalanceInfo *_t = static_cast<BalanceInfo *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->addr != *reinterpret_cast< QString*>(_v)) {
                _t->addr = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 1:
            if (_t->human != *reinterpret_cast< QString*>(_v)) {
                _t->human = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 2:
            if (_t->decimal != *reinterpret_cast< QString*>(_v)) {
                _t->decimal = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 3:
            if (_t->value != *reinterpret_cast< qreal*>(_v)) {
                _t->value = *reinterpret_cast< qreal*>(_v);
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

const QMetaObject BalanceInfo::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BalanceInfo.data,
      qt_meta_data_BalanceInfo,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BalanceInfo::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BalanceInfo::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BalanceInfo.stringdata0))
        return static_cast<void*>(const_cast< BalanceInfo*>(this));
    return QObject::qt_metacast(_clname);
}

int BalanceInfo::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    
#ifndef QT_NO_PROPERTIES
   if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 4;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
struct qt_meta_stringdata_SimpleChain_t {
    QByteArrayData data[7];
    char stringdata0[50];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SimpleChain_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SimpleChain_t qt_meta_stringdata_SimpleChain = {
    {
QT_MOC_LITERAL(0, 0, 11), // "SimpleChain"
QT_MOC_LITERAL(1, 12, 4), // "hash"
QT_MOC_LITERAL(2, 17, 11), // "permissions"
QT_MOC_LITERAL(3, 29, 3), // "uri"
QT_MOC_LITERAL(4, 33, 2), // "to"
QT_MOC_LITERAL(5, 36, 7), // "content"
QT_MOC_LITERAL(6, 44, 5) // "valid"

    },
    "SimpleChain\0hash\0permissions\0uri\0to\0"
    "content\0valid"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SimpleChain[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       6,   14, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // properties: name, type, flags
       1, QMetaType::QString, 0x00095003,
       2, QMetaType::QString, 0x00095003,
       3, QMetaType::QString, 0x00095003,
       4, QMetaType::QString, 0x00095003,
       5, QMetaType::QByteArray, 0x00095003,
       6, QMetaType::Bool, 0x00095003,

       0        // eod
};

void SimpleChain::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{

#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty) {
        SimpleChain *_t = static_cast<SimpleChain *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->hash; break;
        case 1: *reinterpret_cast< QString*>(_v) = _t->permissions; break;
        case 2: *reinterpret_cast< QString*>(_v) = _t->uri; break;
        case 3: *reinterpret_cast< QString*>(_v) = _t->to; break;
        case 4: *reinterpret_cast< QByteArray*>(_v) = _t->content; break;
        case 5: *reinterpret_cast< bool*>(_v) = _t->valid; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        SimpleChain *_t = static_cast<SimpleChain *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->hash != *reinterpret_cast< QString*>(_v)) {
                _t->hash = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 1:
            if (_t->permissions != *reinterpret_cast< QString*>(_v)) {
                _t->permissions = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 2:
            if (_t->uri != *reinterpret_cast< QString*>(_v)) {
                _t->uri = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 3:
            if (_t->to != *reinterpret_cast< QString*>(_v)) {
                _t->to = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 4:
            if (_t->content != *reinterpret_cast< QByteArray*>(_v)) {
                _t->content = *reinterpret_cast< QByteArray*>(_v);
            }
            break;
        case 5:
            if (_t->valid != *reinterpret_cast< bool*>(_v)) {
                _t->valid = *reinterpret_cast< bool*>(_v);
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

const QMetaObject SimpleChain::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SimpleChain.data,
      qt_meta_data_SimpleChain,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SimpleChain::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SimpleChain::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SimpleChain.stringdata0))
        return static_cast<void*>(const_cast< SimpleChain*>(this));
    return QObject::qt_metacast(_clname);
}

int SimpleChain::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    
#ifndef QT_NO_PROPERTIES
   if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 6;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 6;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 6;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 6;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 6;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
struct qt_meta_stringdata_BWView_t {
    QByteArrayData data[6];
    char stringdata0[62];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BWView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BWView_t qt_meta_stringdata_BWView = {
    {
QT_MOC_LITERAL(0, 0, 6), // "BWView"
QT_MOC_LITERAL(1, 7, 17), // "interfacesChanged"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 15), // "servicesChanged"
QT_MOC_LITERAL(4, 42, 10), // "interfaces"
QT_MOC_LITERAL(5, 53, 8) // "services"

    },
    "BWView\0interfacesChanged\0\0servicesChanged\0"
    "interfaces\0services"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BWView[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       2,   26, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x06 /* Public */,
       3,    0,   25,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

 // properties: name, type, flags
       4, QMetaType::QVariantList, 0x00495001,
       5, QMetaType::QStringList, 0x00495001,

 // properties: notify_signal_id
       0,
       1,

       0        // eod
};

void BWView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BWView *_t = static_cast<BWView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->interfacesChanged(); break;
        case 1: _t->servicesChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (BWView::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BWView::interfacesChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (BWView::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BWView::servicesChanged)) {
                *result = 1;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        BWView *_t = static_cast<BWView *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QVariantList*>(_v) = _t->interfaces(); break;
        case 1: *reinterpret_cast< QStringList*>(_v) = _t->services(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
    Q_UNUSED(_a);
}

const QMetaObject BWView::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BWView.data,
      qt_meta_data_BWView,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BWView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BWView::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BWView.stringdata0))
        return static_cast<void*>(const_cast< BWView*>(this));
    return QObject::qt_metacast(_clname);
}

int BWView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
#ifndef QT_NO_PROPERTIES
   else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
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

// SIGNAL 0
void BWView::interfacesChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void BWView::servicesChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
