cmake_minimum_required(VERSION 3.1.0)
project(CellMate_Standalone)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search)
find_package(RTABMap REQUIRED)

# TODO Move BW to library
set(BW_INCLUDE_DIRS "/opt/Qt5.7.0/5.7/gcc_64/qml/io/bw2/")
set(BW_LIBRARIES "/opt/Qt5.7.0/5.7/gcc_64/qml/io/bw2/libbosswave.so")

set(CellMate_LIB_PATH "${PROJECT_SOURCE_DIR}/../lib/")
set(CellMate_INCLUDE_DIRS ${CellMate_LIB_PATH} "${PROJECT_SOURCE_DIR}/src/")
set(CellMate_LIBRARIES cellmate)

set(INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${RTABMap_INCLUDE_DIRS}
    ${MHD_INCLUDE_DIRS}
    ${BW_INCLUDE_DIRS}
)

set(LIBRARIES
    ${Qt5Core_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${BW_LIBRARIES}
    ${CellMate_LIBRARIES}
)

include_directories(SYSTEM ${INCLUDE_DIRS}) # to avoid warnings from other libraries
include_directories(${CellMate_INCLUDE_DIRS})

set(SOURCES
    "${PROJECT_SOURCE_DIR}/src/front_end/bosswave/BWServer.cpp"
    "${PROJECT_SOURCE_DIR}/src/front_end/bosswave/BWWorker.cpp"
    "${PROJECT_SOURCE_DIR}/src/front_end/http/HTTPFrontEndObj.cpp"
    "${PROJECT_SOURCE_DIR}/src/event/QueryEvent.cpp"
    "${PROJECT_SOURCE_DIR}/src/event/DetectionEvent.cpp"
    "${PROJECT_SOURCE_DIR}/src/event/FailureEvent.cpp"
    "${PROJECT_SOURCE_DIR}/src/process/IdentificationObj.cpp"
    "${PROJECT_SOURCE_DIR}/src/main.cpp")

add_executable(cellmate_standalone ${SOURCES})

target_link_libraries(cellmate_standalone ${LIBRARIES})

set_property(TARGET cellmate_standalone PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
