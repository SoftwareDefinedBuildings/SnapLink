cmake_minimum_required(VERSION 3.1.0)
project(CellMate)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search)
find_package(RTABMap REQUIRED)

set(MHD_INCLUDE_DIRS "/usr/include/")
set(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")

set(BW_INCLUDE_DIRS "/opt/Qt5.7.0/5.7/gcc_64/qml/io/bw2/")
set(BW_LIBRARIES "/opt/Qt5.7.0/5.7/gcc_64/qml/io/bw2/libbosswave.so")

set(CellMate_INCLUDE_DIRS "../src" "src")

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

include_directories(SYSTEM ${INCLUDE_DIRS})
include_directories(${CellMate_INCLUDE_DIRS})

set(SOURCES
    "../src/util/Time.cpp"
    "../src/util/Utility.cpp"
    "../src/adapter/RTABMapDBAdapter.cpp"
    "../src/data/Transform.cpp"
    "../src/data/LabelsSimple.cpp"
    "../src/data/Label.cpp"
    "../src/data/Word.cpp"
    "../src/data/CameraModel.cpp"
    "../src/data/WordsKdTree.cpp"
    "../src/algo/WordSearch.cpp"
    "../src/algo/Visibility.cpp"
    "../src/algo/Feature.cpp"
    "../src/algo/Perspective.cpp"
    "src/front/BWServer.cpp"
    "src/front/BWWorker.cpp"
    "src/front/HTTPServer.cpp"
    "src/event/QueryEvent.cpp"
    "src/event/DetectionEvent.cpp"
    "src/event/FailureEvent.cpp"
    "src/process/Identification.cpp"
    "src/main.cpp")

add_executable(cellmate ${SOURCES})

target_link_libraries(cellmate ${LIBRARIES})

set_property(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
