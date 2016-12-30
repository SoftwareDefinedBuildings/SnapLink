cmake_minimum_required(VERSION 3.1.0)

project(CellMate_Distributed)

set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

find_package(Protobuf REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search)
find_package(RTABMap REQUIRED)

set(MHD_INCLUDE_DIRS "/usr/include/")

set(BW_INCLUDE_DIRS "/opt/Qt5.7.0/5.7/gcc_64/qml/io/bw2/")
set(BW_LIBRARIES "/opt/Qt5.7.0/5.7/gcc_64/qml/io/bw2/libbosswave.so")

set(GRPC_LIBRARIES grpc++_unsecure grpc gpr)

set(CellMate_LIB_PATH "${PROJECT_SOURCE_DIR}/../lib/")
set(CellMate_INCLUDE_DIRS ${CellMate_LIB_PATH} ${PROJECT_SOURCE_DIR})
set(CellMate_LIBRARIES cellmate)

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
    ${PROTOBUF_LIBRARY}
    ${Qt5Core_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${GRPC_LIBRARIES}
    ${BW_LIBRARIES}
    ${CellMate_LIBRARIES}
)

include_directories(SYSTEM ${INCLUDE_DIRS}) # to avoid warnings from other libraries
include_directories(${CellMate_INCLUDE_DIRS})

# Protobuf
set(PROTO_PATH "${PROJECT_SOURCE_DIR}/proto")
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
set(GRPC_PATH "${PROJECT_SOURCE_DIR}/grpc")
set(GRPC_FILES "${GRPC_PATH}/*.proto")

set(GENERATED_GRPC_PATH "${CMAKE_BINARY_DIR}/generated/grpc")
file(MAKE_DIRECTORY ${GENERATED_GRPC_PATH})
include_directories(SYSTEM ${GENERATED_GRPC_PATH})
set(GENERATED_GRPC_SOURCES
    "${GENERATED_GRPC_PATH}/FrontEndService.pb.h"
    "${GENERATED_GRPC_PATH}/FrontEndService.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontEndService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/FrontEndService.grpc.pb.cc"
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
    "${PROJECT_SOURCE_DIR}/front_end/bosswave/main3.cpp")
add_executable(bw_front ${BWServer_SOURCES})
target_link_libraries(bw_front ${LIBRARIES})
set_property(TARGET bw_front PROPERTY CXX_STANDARD 11)

# bwFront
#set(BWServer_SOURCES
#    "../src/util/Time.cpp"
#    "../src/data/CameraModel.cpp"
#    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.cc"
#    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
#    "${GENERATED_GRPC_PATH}/FrontEndService.pb.cc"
#    "${GENERATED_GRPC_PATH}/FrontEndService.grpc.pb.cc"
#    "${GENERATED_GRPC_PATH}/FeatureService.pb.cc"
#    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.cc"
#    "${PROJECT_SOURCE_DIR}/src/front/BWServer.cpp"
#    "${PROJECT_SOURCE_DIR}/src/front/FeatureClient.cpp"
#    "${PROJECT_SOURCE_DIR}/src/front/main2.cpp")

#add_executable(bwfront ${BWServer_SOURCES})

#target_link_libraries(bwfront ${LIBRARIES})

#set_property(TARGET bwfront PROPERTY CXX_STANDARD 11)



# Front
set(HTTPFrontEnd_SOURCES
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/QueryMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontEndService.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontEndService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.pb.cc"
    "${GENERATED_GRPC_PATH}/FeatureService.grpc.pb.cc"
    "${PROJECT_SOURCE_DIR}/front_end/http/HTTPFrontEndServer.cpp"
    "${PROJECT_SOURCE_DIR}/front_end/http/FeatureClient.cpp"
    "${PROJECT_SOURCE_DIR}/front_end/http/main.cpp")

add_executable(http_front ${HTTPFrontEnd_SOURCES})

target_link_libraries(http_front ${LIBRARIES})

set_property(TARGET http_front PROPERTY CXX_STANDARD 11)

# Feature
set(Feature_SOURCES
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
    "${PROJECT_SOURCE_DIR}/process/feature/FeatureServer.cpp"
    "${PROJECT_SOURCE_DIR}/process/feature/WordSearchClient.cpp"
    "${PROJECT_SOURCE_DIR}/process/feature/main.cpp")

add_executable(feature ${Feature_SOURCES})

target_link_libraries(feature ${LIBRARIES})

set_property(TARGET feature PROPERTY CXX_STANDARD 11)

# WordSearch
set(WordSearch_SOURCES
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
    "${PROJECT_SOURCE_DIR}/process/wordSearch/WordSearchServer.cpp"
    "${PROJECT_SOURCE_DIR}/process/wordSearch/PerspectiveClient.cpp"
    "${PROJECT_SOURCE_DIR}/process/wordSearch/main.cpp")

add_executable(wordsearch ${WordSearch_SOURCES})

target_link_libraries(wordsearch ${LIBRARIES})

set_property(TARGET wordsearch PROPERTY CXX_STANDARD 11)


# Perspective
set(Perspective_SOURCES
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
    "${PROJECT_SOURCE_DIR}/process/perspective/PerspectiveServer.cpp"
    "${PROJECT_SOURCE_DIR}/process/perspective/VisibilityClient.cpp"
    "${PROJECT_SOURCE_DIR}/process/perspective/main.cpp")

add_executable(perspective ${Perspective_SOURCES})

target_link_libraries(perspective ${LIBRARIES})

set_property(TARGET perspective PROPERTY CXX_STANDARD 11)

# Visibility
set(Visibility_SOURCES
    "${GENERATED_PROTOBUF_PATH}/Empty.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/CameraModel.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Transform.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/Session.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/LocationMessage.pb.cc"
    "${GENERATED_PROTOBUF_PATH}/DetectionMessage.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.pb.cc"
    "${GENERATED_GRPC_PATH}/VisibilityService.grpc.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontEndService.pb.cc"
    "${GENERATED_GRPC_PATH}/FrontEndService.grpc.pb.cc"
    "${PROJECT_SOURCE_DIR}/process/visibility/VisibilityServer.cpp"
    "${PROJECT_SOURCE_DIR}/process/visibility/HTTPFrontEndClient.cpp"
    "${PROJECT_SOURCE_DIR}/process/visibility/main.cpp")

add_executable(visibility ${Visibility_SOURCES})

target_link_libraries(visibility ${LIBRARIES})

set_property(TARGET visibility PROPERTY CXX_STANDARD 11)
