QT += qml quick
CONFIG += qt plugin c++11

INCLUDEPATH += $$PWD 

DEFINES += ED25519_REFHASH=1
DEFINES += ED25519_CUSTOMRANDOM=1

SOURCES += \
    $$PWD/bosswave_plugin.cpp \
    $$PWD/bosswave.cpp \
    $$PWD/libbw.cpp \
    $$PWD/agentconnection.cpp \
    $$PWD/message.cpp \
    $$PWD/crypto.cpp \
    $$PWD/ed25519/ed25519.c

HEADERS += \
    $$PWD/bosswave_plugin.h \
    $$PWD/bosswave.h \
    $$PWD/libbw.h \
    $$PWD/utils.h \
    $$PWD/agentconnection.h \
    $$PWD/allocations.h \
    $$PWD/message.h \
    $$PWD/crypto.h

include($$PWD/vendor/qmsgpack/qmsgpack.pri)
