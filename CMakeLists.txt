cmake_minimum_required(VERSION 3.1.0)
project(CellMate)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS viz core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search io filters)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(RTABMap REQUIRED)

set(MHD_INCLUDE_DIRS "/usr/include/")
set(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")

set(BW_INCLUDE_DIRS "/usr/local/Qt-5.8.0/qml/io/bw2/")
set(BW_LIBRARIES "/usr/local/Qt-5.8.0/qml/io/bw2/libbosswave.so")

set(INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${RTABMap_INCLUDE_DIRS}
    ${MHD_INCLUDE_DIRS}
    ${BW_INCLUDE_DIRS}
)

set(LIBRARIES
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${RTABMap_LIBRARIES}
    ${MHD_LIBRARIES}
    ${BW_LIBRARIES}
)

set(CellMate_SOURCE_DIR "${PROJECT_SOURCE_DIR}/src/")

set(SOURCES
    "${CellMate_SOURCE_DIR}/lib/front_end/http/HTTPFrontEnd.cpp"
    "${CellMate_SOURCE_DIR}/lib/front_end/bosswave/BWFrontEnd.cpp"
    "${CellMate_SOURCE_DIR}/lib/front_end/bosswave/BWWorker.cpp"
    "${CellMate_SOURCE_DIR}/lib/util/Utility.cpp"
    "${CellMate_SOURCE_DIR}/lib/adapter/rtabmap/RTABMapAdapter.cpp"
    "${CellMate_SOURCE_DIR}/lib/data/Transform.cpp"
    "${CellMate_SOURCE_DIR}/lib/data/Label.cpp"
    "${CellMate_SOURCE_DIR}/lib/data/Word.cpp"
    "${CellMate_SOURCE_DIR}/lib/data/Room.cpp"
    "${CellMate_SOURCE_DIR}/lib/data/Image.cpp"
    "${CellMate_SOURCE_DIR}/lib/data/CameraModel.cpp"
    "${CellMate_SOURCE_DIR}/lib/algo/WordSearch.cpp"
    "${CellMate_SOURCE_DIR}/lib/algo/RoomSearch.cpp"
    "${CellMate_SOURCE_DIR}/lib/algo/Visibility.cpp"
    "${CellMate_SOURCE_DIR}/lib/algo/Feature.cpp"
    "${CellMate_SOURCE_DIR}/lib/algo/Perspective.cpp"
    "${CellMate_SOURCE_DIR}/run/run.cpp"
    "${CellMate_SOURCE_DIR}/label/res/label_dot.qrc"
    "${CellMate_SOURCE_DIR}/label/widget.cpp"
    "${CellMate_SOURCE_DIR}/vis/vis.cpp"
    "${CellMate_SOURCE_DIR}/label/label.cpp"
    "${CellMate_SOURCE_DIR}/main.cpp")

include_directories(SYSTEM ${INCLUDE_DIRS}) # to avoid warnings from other libraries
include_directories(${CellMate_SOURCE_DIR})

add_executable(cellmate ${SOURCES})

target_link_libraries(cellmate ${LIBRARIES})

set_property(TARGET cellmate PROPERTY CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
