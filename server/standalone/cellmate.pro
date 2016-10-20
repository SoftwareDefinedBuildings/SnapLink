CONFIG += qt
QT -= gui

CONFIG += c++11
CONFIG += console
CONFIG += app_bundle

TEMPLATE = app
# /usr/include must be before /usr/local/include/opencv2 because they both contain flann/flann.cpp that are used
INCLUDEPATH += /usr/include/
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lmicrohttpd

INCLUDEPATH += /usr/local/include/opencv2
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system

INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /usr/local/include/opencv2
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_features2d
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_calib3d
LIBS += -lopencv_flann
LIBS += -lopencv_xfeatures2d

INCLUDEPATH += /usr/local/include/pcl-1.8
LIBS += -L/usr/local/lib
LIBS += -lpcl_common
LIBS += -lpcl_search

INCLUDEPATH += /usr/local/include/rtabmap-0.11
LIBS += -L/usr/local/lib
LIBS += -lrtabmap_core
LIBS += -lrtabmap_utilite

INCLUDEPATH += ../lib
INCLUDEPATH += src

SOURCES += ../lib/util/Time.cpp
SOURCES += ../lib/util/Utility.cpp
SOURCES += ../lib/adapter/RTABMapDBAdapter.cpp
SOURCES += ../lib/data/Transform.cpp
SOURCES += ../lib/data/LabelsSimple.cpp
SOURCES += ../lib/data/Label.cpp
SOURCES += ../lib/data/Word.cpp
SOURCES += ../lib/data/CameraModel.cpp
SOURCES += ../lib/data/WordsKdTree.cpp
SOURCES += ../lib/algo/WordSearch.cpp
SOURCES += ../lib/algo/Visibility.cpp
SOURCES += ../lib/algo/Feature.cpp
SOURCES += ../lib/algo/Perspective.cpp
SOURCES += src/front/HTTPServer.cpp
SOURCES += src/event/QueryEvent.cpp
SOURCES += src/event/DetectionEvent.cpp
SOURCES += src/event/FailureEvent.cpp
SOURCES += src/process/Identification.cpp
SOURCES += src/main.cpp

TARGET = cellmate

include(../third_party/qtlibbw/bosswave.pri)
