cmake_minimum_required(VERSION 3.1.0)

PROJECT(CellMate)

SET(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

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

# Protobuf
SET(PROTO_PATH "${CMAKE_SOURCE_DIR}/src/proto")
SET(PROTO_FILES "${PROTO_PATH}/*.proto")

SET(GENERATED_PROTOBUF_PATH "${CMAKE_BINARY_DIR}/generated/proto")
FILE(MAKE_DIRECTORY ${GENERATED_PROTOBUF_PATH})
INCLUDE_DIRECTORIES(SYSTEM ${GENERATED_PROTOBUF_PATH})
SET(GENERATED_PROTO_SOURCES
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.h"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/KeyPoint.pb.h"
    "${GENERATED_PROTOBUF_PATH}/KeyPoint.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Descriptor.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Descriptor.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Transform.pb.h"
    "${GENERATED_PROTOBUF_PATH}/Transform.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/FeatureMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/FeatureMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/WordMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/WordMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/SignatureMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/SignatureMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.cc")

ADD_CUSTOM_COMMAND(
    OUTPUT ${GENERATED_PROTO_SOURCES}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--cpp_out=${GENERATED_PROTOBUF_PATH}"
         "${PROTO_FILES}"
    )

# GRPC
SET(GRPC_CPP_PLUGIN_PATH "/usr/local/bin/grpc_cpp_plugin")
SET(GRPC_PATH "${CMAKE_SOURCE_DIR}/src/grpc")
SET(GRPC_FILES "${GRPC_PATH}/*.proto")

SET(GENERATED_GRPC_PATH "${CMAKE_BINARY_DIR}/generated/grpc")
FILE(MAKE_DIRECTORY ${GENERATED_GRPC_PATH})
INCLUDE_DIRECTORIES(SYSTEM ${GENERATED_GRPC_PATH})
SET(GENERATED_GRPC_SOURCES
    "${GENERATED_GRPC_PATH}/FrontService.pb.h"
    "${GENERATED_GRPC_PATH}/FrontService.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/FrontService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.pb.h"
    "${GENERATED_GRPC_PATH}/FeatureService.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/WordSearchService.pb.h"
    "${GENERATED_GRPC_PATH}/WordSearchService.pb.cc"
    "${GENERATED_GRPC_PATH}/WordSearchService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/WordSearchService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/SignatureSearchService.pb.h"
    "${GENERATED_GRPC_PATH}/SignatureSearchService.pb.cc"
    "${GENERATED_GRPC_PATH}/SignatureSearchService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/SignatureSearchService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.pb.h"
    "${GENERATED_GRPC_PATH}/PerspectiveService.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/PerspectiveService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.h"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.cc")

ADD_CUSTOM_COMMAND(
    OUTPUT ${GENERATED_GRPC_SOURCES}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--proto_path=${GRPC_PATH}"
         "--grpc_out=${GENERATED_GRPC_PATH}"
         "--plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_PATH}"
         "${GRPC_FILES}"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--proto_path=${GRPC_PATH}"
         "--cpp_out=${GENERATED_GRPC_PATH}"
         "${GRPC_FILES}"
    )

# Front
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
    "src/front/HTTPServer.cpp"
    "src/front/FeatureClient.cpp"
    "src/front/main.cpp")

ADD_EXECUTABLE(front ${HTTPServer_SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(front ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET front PROPERTY CXX_STANDARD 11)

# Feature
SET(Feature_SOURCES
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
    "src/feature/FeatureServer.cpp"
    "src/feature/WordSearchClient.cpp"
    "src/feature/main.cpp")

ADD_EXECUTABLE(feature ${Feature_SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(feature ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET feature PROPERTY CXX_STANDARD 11)

# WordSearch
SET(WordSearch_SOURCES
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
    "src/wordSearch/WordSearchServer.cpp"
    "src/wordSearch/SignatureSearchClient.cpp"
    "src/wordSearch/main.cpp")

ADD_EXECUTABLE(wordsearch ${WordSearch_SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(wordsearch ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET wordsearch PROPERTY CXX_STANDARD 11)

# SignatureSearch
SET(SignatureSearch_SOURCES
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
    "src/signatureSearch/SignatureSearchServer.cpp"
    "src/signatureSearch/PerspectiveClient.cpp"
    "src/signatureSearch/main.cpp")

ADD_EXECUTABLE(signaturesearch ${SignatureSearch_SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(signaturesearch ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET signaturesearch PROPERTY CXX_STANDARD 11)

# Perspective
SET(Perspective_SOURCES
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
    "src/perspective/PerspectiveServer.cpp"
    "src/perspective/VisibilityClient.cpp"
    "src/perspective/main.cpp")

ADD_EXECUTABLE(perspective ${Perspective_SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(perspective ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET perspective PROPERTY CXX_STANDARD 11)

# Visibility
SET(Visibility_SOURCES
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
    "src/visibility/VisibilityServer.cpp"
    "src/visibility/HTTPClient.cpp"
    "src/visibility/main.cpp")

ADD_EXECUTABLE(visibility ${Visibility_SOURCES} ${GENERATED_PROTO_SOURCES} ${GENERATED_GRPC_SOURCES})

TARGET_LINK_LIBRARIES(visibility ${LIBRARIES} grpc++_unsecure grpc gpr)

SET_PROPERTY(TARGET visibility PROPERTY CXX_STANDARD 11)
