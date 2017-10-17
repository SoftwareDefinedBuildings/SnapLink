cmake_minimum_required(VERSION 3.1.0)
project(SnapLink)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Protobuf REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS viz core imgproc imgcodecs calib3d flann xfeatures2d)
find_package(PCL 1.8 REQUIRED COMPONENTS common search io filters)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(RTABMap REQUIRED)

set(ZBAR_LIBRARIES zbar)
set(GRPC_LIBRARIES grpc++_unsecure grpc gpr)
set(APRILTAG_LIBRARIES apriltags)

set(INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${RTABMap_INCLUDE_DIRS}
)

set(LIBRARIES
    ${ZBAR_LIBRARIES}
    ${PROTOBUF_LIBRARY}
    ${GRPC_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Concurrent_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${RTABMap_LIBRARIES}
    ${APRILTAG_LIBRARIES}
)

set(SnapLink_SOURCE_DIR "${PROJECT_SOURCE_DIR}/src/")

include_directories(SYSTEM ${INCLUDE_DIRS}) # to avoid warnings from other libraries
include_directories(${SnapLink_SOURCE_DIR})

# GRPC
set(GRPC_CPP_PLUGIN_PATH "/usr/local/bin/grpc_cpp_plugin")
set(GRPC_PATH "${PROJECT_SOURCE_DIR}/src/proto")
set(GRPC_FILES "${GRPC_PATH}/*.proto")

set(GENERATED_GRPC_PATH "${CMAKE_BINARY_DIR}/generated/grpc")
file(MAKE_DIRECTORY ${GENERATED_GRPC_PATH})
include_directories(SYSTEM ${GENERATED_GRPC_PATH})
set(GENERATED_GRPC_SOURCES
    "${GENERATED_GRPC_PATH}/GrpcService.pb.h"
    "${GENERATED_GRPC_PATH}/GrpcService.pb.cc"
    "${GENERATED_GRPC_PATH}/GrpcService.grpc.pb.h"
    "${GENERATED_GRPC_PATH}/GrpcService.grpc.pb.cc")

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

set(SOURCES
    "${GENERATED_GRPC_PATH}/GrpcService.pb.cc"
    "${GENERATED_GRPC_PATH}/GrpcService.grpc.pb.cc"
    "${SnapLink_SOURCE_DIR}/lib/front_end/grpc/GrpcFrontEnd.cpp"
    "${SnapLink_SOURCE_DIR}/lib/util/Utility.cpp"
    "${SnapLink_SOURCE_DIR}/lib/adapter/rtabmap/RTABMapAdapter.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/Transform.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/Label.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/Word.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/Room.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/Image.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/FoundItem.cpp"
    "${SnapLink_SOURCE_DIR}/lib/data/CameraModel.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/WordSearch.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/RoomSearch.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/Visibility.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/Feature.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/Perspective.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/WordCluster.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/Apriltag.cpp"
    "${SnapLink_SOURCE_DIR}/lib/algo/QR.cpp"
    "${SnapLink_SOURCE_DIR}/lib/visualize/visualize.cpp"
    "${SnapLink_SOURCE_DIR}/run/Run.cpp"
    "${SnapLink_SOURCE_DIR}/label/res/label_dot.qrc"
    "${SnapLink_SOURCE_DIR}/label/Widget.cpp"
    "${SnapLink_SOURCE_DIR}/vis/Visualizer.cpp"
    "${SnapLink_SOURCE_DIR}/label/Labeler.cpp"
    "${SnapLink_SOURCE_DIR}/measure/Measure.cpp"
    "${SnapLink_SOURCE_DIR}/measure/MeasureWidget.cpp"
    "${SnapLink_SOURCE_DIR}/main.cpp")

add_executable(snaplink ${SOURCES})

target_link_libraries(snaplink ${LIBRARIES})

set_property(TARGET snaplink PROPERTY CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
