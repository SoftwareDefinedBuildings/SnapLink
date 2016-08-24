CONFIG += qt
QT -= gui

CONFIG += c++11

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

INCLUDEPATH += /usr/local/include/rtabmap-0.11
LIBS += -L/usr/local/lib
LIBS += -lrtabmap_core
LIBS += -lrtabmap_utilite

INCLUDEPATH += /usr/include/
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lmicrohttpd

INCLUDEPATH += ../../core

SOURCES += ../../core/data/CameraModel.cpp
SOURCES += ../../core/data/Transform.cpp
SOURCES += ../../core/data/LabelsSimple.cpp
SOURCES += ../../core/data/Word.cpp
SOURCES += ../../core/data/Signature.cpp
SOURCES += ../../core/data/WordsKdTree.cpp
SOURCES += ../../core/data/Label.cpp
SOURCES += ../../core/data/SignaturesSimple.cpp
SOURCES += ../../core/adapter/RTABMapDBAdapter.cpp
SOURCES += ../../core/event/FailureEvent.cpp
SOURCES += ../../core/event/WordEvent.cpp
SOURCES += ../../core/event/FeatureEvent.cpp
SOURCES += ../../core/event/QueryEvent.cpp
SOURCES += ../../core/event/LocationEvent.cpp
SOURCES += ../../core/event/DetectionEvent.cpp
SOURCES += ../../core/event/SignatureEvent.cpp
SOURCES += ../../core/front/HTTPServer.cpp
SOURCES += ../../core/stage/FeatureExtraction.cpp
SOURCES += ../../core/stage/Visibility.cpp
SOURCES += ../../core/stage/WordSearch.cpp
SOURCES += ../../core/stage/Perspective.cpp
SOURCES += ../../core/stage/SignatureSearch.cpp
SOURCES += ../../core/util/Utility.cpp
SOURCES += ../../core/util/Time.cpp
SOURCES += src/main.cpp

TARGET = cellmate
