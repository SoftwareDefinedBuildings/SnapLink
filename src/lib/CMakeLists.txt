cmake_minimum_required(VERSION 3.1.0)
project(CellMate_Library)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search)
find_package(RTABMap REQUIRED)

# TODO make these parameters
set(MHD_INCLUDE_DIRS "/usr/include/")
set(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")
set(BW_INCLUDE_DIRS "/usr/local/Qt-5.8.0/qml/io/bw2/")
set(BW_LIBRARIES "/usr/local/Qt-5.8.0/qml/io/bw2/libbosswave.so")

set(CellMate_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}")

set(INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${RTABMap_INCLUDE_DIRS}
    ${MHD_INCLUDE_DIRS}
    ${BW_INCLUDE_DIRS}
)

set(LIBRARIES
    ${Qt5Core_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${RTABMap_LIBRARIES}
    ${MHD_LIBRARIES}
    ${BW_LIBRARIES}
)

include_directories(SYSTEM ${INCLUDE_DIRS}) # to avoid warnings from other libraries
include_directories(${CellMate_INCLUDE_DIRS})

set(SOURCES
    "${PROJECT_SOURCE_DIR}/front_end/http/HTTPFrontEnd.cpp"
    "${PROJECT_SOURCE_DIR}/front_end/bosswave/BWFrontEnd.cpp"
    "${PROJECT_SOURCE_DIR}/front_end/bosswave/BWWorker.cpp"
    "${PROJECT_SOURCE_DIR}/util/Utility.cpp"
    "${PROJECT_SOURCE_DIR}/adapter/rtabmap/RTABMapDBAdapter.cpp"
    "${PROJECT_SOURCE_DIR}/data/Transform.cpp"
    "${PROJECT_SOURCE_DIR}/data/LabelsSimple.cpp"
    "${PROJECT_SOURCE_DIR}/data/Label.cpp"
    "${PROJECT_SOURCE_DIR}/data/Word.cpp"
    "${PROJECT_SOURCE_DIR}/data/CameraModel.cpp"
    "${PROJECT_SOURCE_DIR}/data/WordsKdTree.cpp"
    "${PROJECT_SOURCE_DIR}/algo/WordSearch.cpp"
    "${PROJECT_SOURCE_DIR}/algo/DbSearch.cpp"
    "${PROJECT_SOURCE_DIR}/algo/Visibility.cpp"
    "${PROJECT_SOURCE_DIR}/algo/Feature.cpp"
    "${PROJECT_SOURCE_DIR}/algo/Perspective.cpp")

add_library(cellmate ${SOURCES})

target_link_libraries(cellmate ${LIBRARIES})

set_property(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
