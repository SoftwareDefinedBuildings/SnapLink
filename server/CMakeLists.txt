cmake_minimum_required(VERSION 3.1.0)
PROJECT(CellMate)

SET(CMAKE_AUTOMOC ON)

FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
FIND_PACKAGE(PCL 1.8 REQUIRED COMPONENTS common)
FIND_PACKAGE(RTABMap REQUIRED)

SET(MHD_INCLUDE_DIRS "/usr/include/")
SET(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")

SET(CellMate_INCLUDE_DIRS "src/")

SET(INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${RTABMap_INCLUDE_DIRS}
    ${MHD_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${Qt5Core_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${RTABMap_LIBRARIES}
    ${MHD_LIBRARIES}
)

INCLUDE_DIRECTORIES(SYSTEM ${INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CellMate_INCLUDE_DIRS})

SET(SOURCES
    "src/main.cpp"
    "src/data/CameraModel.cpp"
    "src/data/SensorData.cpp"
    "src/data/Transform.cpp"
    "src/data/LabelsSimple.cpp"
    "src/data/Word.cpp"
    "src/data/Signature.cpp"
    "src/data/WordsKdTree.cpp"
    "src/data/Label.cpp"
    "src/data/SignaturesSimple.cpp"
    "src/adapter/RTABMapDBAdapter.cpp"
    "src/event/FailureEvent.cpp"
    "src/event/WordEvent.cpp"
    "src/event/FeatureEvent.cpp"
    "src/event/QueryEvent.cpp"
    "src/event/LocationEvent.cpp"
    "src/event/DetectionEvent.cpp"
    "src/event/SignatureEvent.cpp"
    "src/stage/FeatureExtraction.cpp"
    "src/stage/Visibility.cpp"
    "src/stage/WordSearch.cpp"
    "src/stage/Perspective.cpp"
    "src/stage/HTTPServer.cpp"
    "src/stage/SignatureSearch.cpp"
    "src/util/Utility.cpp"
    "src/util/Time.cpp")

ADD_EXECUTABLE(cellmate ${SOURCES})

TARGET_LINK_LIBRARIES(cellmate ${LIBRARIES})

SET_PROPERTY(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
