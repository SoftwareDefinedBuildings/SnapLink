cmake_minimum_required(VERSION 3.1.0)
PROJECT(CellMate)

SET(CMAKE_AUTOMOC ON)

FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
FIND_PACKAGE(PCL 1.8 REQUIRED COMPONENTS common)
FIND_PACKAGE(RTABMap REQUIRED)

SET(MHD_INCLUDE_DIRS "/usr/include/")
SET(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")

SET(CellMate_INCLUDE_DIRS "../lib" "src")

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
    "../lib/util/Time.cpp"
    "../lib/util/Utility.cpp"
    "../lib/adapter/RTABMapDBAdapter.cpp"
    "../lib/data/SignaturesSimple.cpp"
    "../lib/data/Transform.cpp"
    "../lib/data/LabelsSimple.cpp"
    "../lib/data/Label.cpp"
    "../lib/data/Signature.cpp"
    "../lib/data/Word.cpp"
    "../lib/data/CameraModel.cpp"
    "../lib/data/WordsKdTree.cpp"
    "../lib/algo/SignatureSearch.cpp"
    "../lib/algo/WordSearch.cpp"
    "../lib/algo/Visibility.cpp"
    "../lib/algo/Feature.cpp"
    "../lib/algo/Perspective.cpp"
    "src/front/HTTPServer.cpp"
    "src/event/QueryEvent.cpp"
    "src/event/WordEvent.cpp"
    "src/event/FeatureEvent.cpp"
    "src/event/LocationEvent.cpp"
    "src/event/SignatureEvent.cpp"
    "src/event/DetectionEvent.cpp"
    "src/event/FailureEvent.cpp"
    "src/stage/VisibilityStage.cpp"
    "src/stage/WordSearchStage.cpp"
    "src/stage/SignatureSearchStage.cpp"
    "src/stage/PerspectiveStage.cpp"
    "src/stage/FeatureStage.cpp"
    "src/main.cpp")

ADD_EXECUTABLE(cellmate ${SOURCES})

TARGET_LINK_LIBRARIES(cellmate ${LIBRARIES})

SET_PROPERTY(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
