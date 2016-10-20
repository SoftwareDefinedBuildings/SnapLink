QT += core
QT -= gui

CONFIG += c++11

TARGET = bwtest
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

INCLUDEPATH += ../third_party/qtlibbw

include(../third_party/qtlibbw/bosswave.pri)
