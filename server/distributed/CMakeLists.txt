cmake_minimum_required(VERSION 3.1.0)
PROJECT(CellMate)

SET(CMAKE_AUTOMOC ON)

find_package(Protobuf REQUIRED)
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
    ${PROTOBUF_LIBRARY}
    ${Qt5Core_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${RTABMap_LIBRARIES}
    ${MHD_LIBRARIES}
)

INCLUDE_DIRECTORIES(SYSTEM ${INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CellMate_INCLUDE_DIRS})

SET(PROTO_PATH "src/message")

SET(GENERATED_PROTO_SOURCES
    "src/message/CameraModel.pb.h"
    "src/message/CameraModel.pb.cc"
    "src/message/Session.pb.h"
    "src/message/Session.pb.cc"
    "src/message/Query.pb.h"
    "src/message/Query.pb.cc")

SET(PROTO_FILES
    "src/message/CameraModel.proto"
    "src/message/Session.proto"
    "src/message/Query.proto")

ADD_CUSTOM_COMMAND(
    OUTPUT ${GENERATED_PROTO_SOURCES}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--cpp_out=${PROTO_PATH}"
         "${PROTO_FILES}"
    )

SET(GENERATED_GRPC_SOURCES
    "src/front/HTTPServer.grpc.pb.h"
    "src/front/HTTPServer.grpc.pb.cc")

SET(GPRC_FILES
    "src/front/HTTPServer.proto")

ADD_CUSTOM_COMMAND(
    OUTPUT  ${GENERATED_GRPC_SOURCES}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--grpc_out=${PROTO_PATH}"
         "--plugin=protoc-gen-grpc=`which grpc_cpp_plugin`"
         "${GPRC_FILES}"
    )

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
    "src/message/QueryEvent.cpp"
    "src/message/WordEvent.cpp"
    "src/message/FeatureEvent.cpp"
    "src/message/LocationEvent.cpp"
    "src/message/SignatureEvent.cpp"
    "src/message/DetectionEvent.cpp"
    "src/message/FailureEvent.cpp"
    "src/service/VisibilityStage.cpp"
    "src/service/WordSearchStage.cpp"
    "src/service/SignatureSearchStage.cpp"
    "src/service/PerspectiveStage.cpp"
    "src/service/FeatureStage.cpp"
    "src/main.cpp")

ADD_EXECUTABLE(cellmate ${SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(cellmate ${LIBRARIES})

SET_PROPERTY(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
