cmake_minimum_required(VERSION 3.1.0)

project(CellMate)

set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

find_package(Protobuf REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search)
find_package(RTABMap REQUIRED)

set(MHD_INCLUDE_DIRS "/usr/include/")
set(MHD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmicrohttpd.so")

set(BW_INCLUDE_DIRS "/root/workspace/qtlibbw/")
set(BW_LIBRARIES "/usr/local/Qt-5.7.1/qml/io/bw2/libbosswave.so")

set(GRPC_LIBRARIES grpc++_unsecure grpc gpr)

set(CellMate_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/../src" "src")

set(INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${RTABMap_INCLUDE_DIRS}
    ${MHD_INCLUDE_DIRS}
    ${BW_INCLUDE_DIRS}
    ${BW_INCLUDE_DIRS}
)

set(LIBRARIES
    ${PROTOBUF_LIBRARY}
    ${Qt5Core_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${RTABMap_LIBRARIES}
    ${MHD_LIBRARIES}
    ${GRPC_LIBRARIES}
    ${BW_LIBRARIES}
)

include_directories(SYSTEM ${INCLUDE_DIRS})
include_directories(${CellMate_INCLUDE_DIRS})

# Protobuf
set(PROTO_PATH "${CMAKE_SOURCE_DIR}/src/proto")
set(PROTO_FILES "${PROTO_PATH}/*.proto")

set(GENERATED_PROTOBUF_PATH "${CMAKE_BINARY_DIR}/generated/proto")
file(MAKE_DIRECTORY ${GENERATED_PROTOBUF_PATH})
include_directories(SYSTEM ${GENERATED_PROTOBUF_PATH})
set(GENERATED_PROTO_SOURCES
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
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.h"
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.cc")

set_source_files_properties(${GENERATED_PROTO_SOURCES} PROPERTIES COMPILE_FLAGS "-w")

add_custom_command(
    OUTPUT ${GENERATED_PROTO_SOURCES}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS "--proto_path=${PROTO_PATH}"
         "--cpp_out=${GENERATED_PROTOBUF_PATH}"
         "${PROTO_FILES}"
    )

# GRPC
set(GRPC_CPP_PLUGIN_PATH "/usr/local/bin/grpc_cpp_plugin")
set(GRPC_PATH "${CMAKE_SOURCE_DIR}/src/grpc")
set(GRPC_FILES "${GRPC_PATH}/*.proto")

set(GENERATED_GRPC_PATH "${CMAKE_BINARY_DIR}/generated/grpc")
file(MAKE_DIRECTORY ${GENERATED_GRPC_PATH})
include_directories(SYSTEM ${GENERATED_GRPC_PATH})
set(GENERATED_GRPC_SOURCES
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
    "${GENERATED_GRPC_PATH}/PerspectiveService.pb.h"
    "${GENERATED_GRPC_PATH}/PerspectiveService.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/PerspectiveService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.h"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.cc")

set_source_files_properties(${GENERATED_GRPC_SOURCES} PROPERTIES COMPILE_FLAGS "-w")

add_custom_command(
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

# bwFront
set(BWServer_SOURCES
    "src/front/main3.cpp")
add_executable(bwfront ${BWServer_SOURCES})
target_link_libraries(bwfront ${LIBRARIES})
set_property(TARGET bwfront PROPERTY CXX_STANDARD 11)

# bwFront
#set(BWServer_SOURCES
#    "../src/util/Time.cpp"
#    "../src/data/CameraModel.cpp"
#    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
#    "${GENERATED_GRPC_PATH}/FrontService.pb.cc"
#    "${GENERATED_GRPC_PATH}/FrontService.grpc.pb.cc"
#    "${GENERATED_GRPC_PATH}/FeatureService.pb.cc"
#    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.cc"
#    "src/front/BWServer.cpp"
#    "src/front/FeatureClient.cpp"
#    "src/front/main2.cpp")

#add_executable(bwfront ${BWServer_SOURCES})

#target_link_libraries(bwfront ${LIBRARIES})

#set_property(TARGET bwfront PROPERTY CXX_STANDARD 11)



# Front
set(HTTPServer_SOURCES
    "../src/util/Time.cpp"
    "../src/data/CameraModel.cpp"
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontService.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.cc"
    "src/front/HTTPServer.cpp"
    "src/front/FeatureClient.cpp"
    "src/front/main.cpp")

add_executable(front ${HTTPServer_SOURCES})

target_link_libraries(front ${LIBRARIES})

set_property(TARGET front PROPERTY CXX_STANDARD 11)

# Feature
set(Feature_SOURCES
    "../src/util/Time.cpp"
    "../src/data/CameraModel.cpp"
    "../src/algo/Feature.cpp"
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/KeyPoint.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Descriptor.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/FeatureMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/WordSearchService.pb.cc"
    "${GENERATED_GRPC_PATH}/WordSearchService.grpc.pb.cc"
    "src/feature/FeatureServer.cpp"
    "src/feature/WordSearchClient.cpp"
    "src/feature/main.cpp")

add_executable(feature ${Feature_SOURCES})

target_link_libraries(feature ${LIBRARIES})

set_property(TARGET feature PROPERTY CXX_STANDARD 11)

# WordSearch
set(WordSearch_SOURCES
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
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/KeyPoint.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Descriptor.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/FeatureMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/WordMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/WordSearchService.pb.cc"
    "${GENERATED_GRPC_PATH}/WordSearchService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.grpc.pb.cc"
    "src/wordSearch/WordSearchServer.cpp"
    "src/wordSearch/PerspectiveClient.cpp"
    "src/wordSearch/main.cpp")

add_executable(wordsearch ${WordSearch_SOURCES})

target_link_libraries(wordsearch ${LIBRARIES})

set_property(TARGET wordsearch PROPERTY CXX_STANDARD 11)


# Perspective
set(Perspective_SOURCES
    "../src/util/Time.cpp"
    "../src/util/Utility.cpp"
    "../src/adapter/RTABMapDBAdapter.cpp"
    "../src/data/Transform.cpp"
    "../src/data/LabelsSimple.cpp"
    "../src/data/Label.cpp"
    "../src/data/Word.cpp"
    "../src/data/CameraModel.cpp"
    "../src/data/WordsKdTree.cpp"
    "../src/algo/Perspective.cpp"
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/KeyPoint.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Transform.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Descriptor.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/WordMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.pb.cc"
    "${GENERATED_GRPC_PATH}/PerspectiveService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.cc"
    "src/perspective/PerspectiveServer.cpp"
    "src/perspective/VisibilityClient.cpp"
    "src/perspective/main.cpp")

add_executable(perspective ${Perspective_SOURCES})

target_link_libraries(perspective ${LIBRARIES})

set_property(TARGET perspective PROPERTY CXX_STANDARD 11)

# Visibility
set(Visibility_SOURCES
    "../src/util/Time.cpp"
    "../src/util/Utility.cpp"
    "../src/adapter/RTABMapDBAdapter.cpp"
    "../src/data/Transform.cpp"
    "../src/data/LabelsSimple.cpp"
    "../src/data/Label.cpp"
    "../src/data/Word.cpp"
    "../src/data/CameraModel.cpp"
    "../src/data/WordsKdTree.cpp"
    "../src/algo/Visibility.cpp"
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Transform.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontService.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontService.grpc.pb.cc"
    "src/visibility/VisibilityServer.cpp"
    "src/visibility/HTTPClient.cpp"
    "src/visibility/main.cpp")

add_executable(visibility ${Visibility_SOURCES})

target_link_libraries(visibility ${LIBRARIES})

set_property(TARGET visibility PROPERTY CXX_STANDARD 11)
