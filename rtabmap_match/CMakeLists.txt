cmake_minimum_required(VERSION 2.8)
PROJECT( MyProject )

FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    "~/workspace/rtabmap/corelib/include/rtabmap/core"
    "~/workspace/rtabmap/corelib/src"
)

SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

file(GLOB_RECURSE MATCH_SOURCES "src/*.cpp")

ADD_EXECUTABLE(rgbd_example ${MATCH_SOURCES})

TARGET_LINK_LIBRARIES(rgbd_example ${LIBRARIES})

