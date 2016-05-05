cmake_minimum_required(VERSION 2.8)
PROJECT( label-tool )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON) # somehow this isn't working
set(CMAKE_AUTORCC ON) # not working too :(

FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(Qt5Gui REQUIRED)
FIND_PACKAGE(Qt5Widgets REQUIRED)

SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    "/root/workspace/rtabmap/corelib/include/rtabmap/core"
    "/root/workspace/rtabmap/corelib/src"
    "/root/workspace/rtabmap/guilib/include/rtabmap/gui"
    "/root/workspace/rtabmap/guilib/src"
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    "../server/src"
)

SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${QT_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

file(GLOB_RECURSE MATCH_SOURCES "src/*.cpp")

ADD_EXECUTABLE(label-tool
    ${MATCH_SOURCES}
    "../server/src/MemoryLoc.cpp"
)

TARGET_LINK_LIBRARIES(label-tool ${LIBRARIES})
