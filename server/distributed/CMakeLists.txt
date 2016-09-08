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

SET(GRPC_CPP_PLUGIN_PATH "/usr/local/bin/grpc_cpp_plugin")
SET(PROTO_PATH "${CMAKE_SOURCE_DIR}/src/proto")
SET(GENERATED_PROTOBUF_PATH "${CMAKE_BINARY_DIR}/generated")
FILE(MAKE_DIRECTORY ${GENERATED_PROTOBUF_PATH})
INCLUDE_DIRECTORIES(SYSTEM ${GENERATED_PROTOBUF_PATH})

SET(PROTO_FILES "${PROTO_PATH}/*.proto") # so protoc will process all of them

SET(GENERATED_PROTO_SOURCES
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.h"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Query.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Query.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Detection.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Detection.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CellMate.grpc.pb.h"
    "${GENERATED_PROTOBUF_PATH}/CellMate.grpc.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/HTTP.grpc.pb.h"
    "${GENERATED_PROTOBUF_PATH}/HTTP.grpc.pb.cc")

ADD_CUSTOM_COMMAND(
    OUTPUT ${GENERATED_PROTO_SOURCES}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--cpp_out=${GENERATED_PROTOBUF_PATH}"
         "${PROTO_FILES}"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--grpc_out=${GENERATED_PROTOBUF_PATH}"
         "--plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_PATH}"
         "${PROTO_FILES}"
    )

SET(HTTPServer_SOURCES
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
    "src/http/HTTPServer.cpp"
    "src/http/CellMateClient.cpp"
    "src/http/main.cpp")

SET(CellMate_SOURCES
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
    "src/service/CellMateServer.cpp"
    "src/service/HTTPClient.cpp"
    "src/service/main.cpp")

ADD_EXECUTABLE(httpserver ${HTTPServer_SOURCES} ${GENERATED_PROTO_SOURCES})

TARGET_LINK_LIBRARIES(httpserver ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET httpserver PROPERTY CXX_STANDARD 11)

ADD_EXECUTABLE(cellmate ${CellMate_SOURCES} ${GENERATED_PROTO_SOURCES})

TARGET_LINK_LIBRARIES(cellmate ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET cellmate PROPERTY CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
