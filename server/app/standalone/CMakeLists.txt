cmake_minimum_required(VERSION 3.1.0)
PROJECT(CellMate)

SET(CMAKE_AUTOMOC ON)

FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
FIND_PACKAGE(PCL 1.8 REQUIRED COMPONENTS common)
FIND_PACKAGE(RTABMap REQUIRED)

SET(MHD_INCLUDE_DIRS "/usr/include/")
SET(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")

SET(CellMate_INCLUDE_DIRS "../../core/")

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
    "../../core/data/CameraModel.cpp"
    "../../core/data/Transform.cpp"
    "../../core/data/LabelsSimple.cpp"
    "../../core/data/Word.cpp"
    "../../core/data/Signature.cpp"
    "../../core/data/WordsKdTree.cpp"
    "../../core/data/Label.cpp"
    "../../core/data/SignaturesSimple.cpp"
    "../../core/adapter/RTABMapDBAdapter.cpp"
    "../../core/event/FailureEvent.cpp"
    "../../core/event/WordEvent.cpp"
    "../../core/event/FeatureEvent.cpp"
    "../../core/event/QueryEvent.cpp"
    "../../core/event/LocationEvent.cpp"
    "../../core/event/DetectionEvent.cpp"
    "../../core/event/SignatureEvent.cpp"
    "../../core/front/HTTPServer.cpp"
    "../../core/stage/FeatureExtraction.cpp"
    "../../core/stage/Visibility.cpp"
    "../../core/stage/WordSearch.cpp"
    "../../core/stage/Perspective.cpp"
    "../../core/stage/SignatureSearch.cpp"
    "../../core/util/Utility.cpp"
    "../../core/util/Time.cpp"
    "src/main.cpp")

ADD_EXECUTABLE(cellmate ${SOURCES})

TARGET_LINK_LIBRARIES(cellmate ${LIBRARIES})

SET_PROPERTY(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
