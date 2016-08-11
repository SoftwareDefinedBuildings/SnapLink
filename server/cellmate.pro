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

INCLUDEPATH += src

HEADERS += src/data/SignaturesSimple.h
HEADERS += src/data/SensorData.h
HEADERS += src/data/Signature.h
HEADERS += src/data/Word.h
HEADERS += src/data/Words.h
HEADERS += src/data/Signatures.h
HEADERS += src/data/WordsKdTree.h
HEADERS += src/data/Transform.h
HEADERS += src/data/Label.h
HEADERS += src/data/Labels.h
HEADERS += src/data/CameraModel.h
HEADERS += src/data/PerfData.h
HEADERS += src/data/LabelsSimple.h
HEADERS += src/adapter/RTABMapDBAdapter.h
HEADERS += src/event/QueryEvent.h
HEADERS += src/event/FeatureEvent.h
HEADERS += src/event/DetectionEvent.h
HEADERS += src/event/LocationEvent.h
HEADERS += src/event/FailureEvent.h
HEADERS += src/event/WordEvent.h
HEADERS += src/event/SignatureEvent.h
HEADERS += src/stage/Perspective.h
HEADERS += src/stage/Visibility.h
HEADERS += src/stage/SignatureSearch.h
HEADERS += src/stage/FeatureExtraction.h
HEADERS += src/stage/WordSearch.h
HEADERS += src/stage/HTTPServer.h
HEADERS += src/util/Utility.h
HEADERS += src/util/Time.h

SOURCES += src/main.cpp
SOURCES += src/data/CameraModel.cpp
SOURCES += src/data/SensorData.cpp
SOURCES += src/data/Transform.cpp
SOURCES += src/data/LabelsSimple.cpp
SOURCES += src/data/Word.cpp
SOURCES += src/data/Signature.cpp
SOURCES += src/data/WordsKdTree.cpp
SOURCES += src/data/Label.cpp
SOURCES += src/data/SignaturesSimple.cpp
SOURCES += src/adapter/RTABMapDBAdapter.cpp
SOURCES += src/event/FailureEvent.cpp
SOURCES += src/event/WordEvent.cpp
SOURCES += src/event/FeatureEvent.cpp
SOURCES += src/event/QueryEvent.cpp
SOURCES += src/event/LocationEvent.cpp
SOURCES += src/event/DetectionEvent.cpp
SOURCES += src/event/SignatureEvent.cpp
SOURCES += src/stage/FeatureExtraction.cpp
SOURCES += src/stage/Visibility.cpp
SOURCES += src/stage/WordSearch.cpp
SOURCES += src/stage/Perspective.cpp
SOURCES += src/stage/HTTPServer.cpp
SOURCES += src/stage/SignatureSearch.cpp
SOURCES += src/util/Utility.cpp
SOURCES += src/util/Time.cpp

TARGET = cellmate
