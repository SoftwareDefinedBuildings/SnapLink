#!/usr/bin/env bash

if [ $# -eq 0 ]; then
  WORKSPACE=$HOME/workspace
else
  WORKSPACE=$1
fi

apt-get update

set -e

## Install prerequisites
apt-get install -y git cmake build-essential

## Qt5
# ref: https://wiki.qt.io/Building_Qt_5_from_Git
# Install prerequisites
apt-get build-dep -y qt5-default
apt-get install -y libxcb-xinerama0-dev

# Clone qt5, init repository, checkout to the target tag (only after init-repository has been run), and get all submodules
git clone https://code.qt.io/qt/qt5.git $WORKSPACE/qt5
cd $WORKSPACE/qt5
perl init-repository
git checkout tags/v5.8.0
git submodule update

# Generate Makefile for open source project
mkdir -p $WORKSPACE/qt5/build
cd $WORKSPACE/qt5/build
../configure -opensource -confirm-license -nomake examples -nomake tests

# Compile and install QT to /usr/local/Qt-%VERSION%
make -j $(nproc)
make install -j $(nproc)

# Add QT binaries to PATH
QT_PATH=/usr/local/Qt-5.8.0
echo "export PATH=$PATH:$QT_PATH/bin" >> $HOME/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$QT_PATH/lib" >> $HOME/.bashrc
source $HOME/.bashrc

## VTK
#Ref: http://www.vtk.org/Wiki/VTK/Building/Linux
# Install prerequisites
apt-get install -y libxt-dev

# Clone VTK to the traget tag
git clone -b v7.1.1 --single-branch --depth 1 https://github.com/Kitware/VTK $WORKSPACE/VTK

# Compile and install VTK with Qt
mkdir -p $WORKSPACE/VTK/build
cd $WORKSPACE/VTK/build
cmake -DCMAKE_BUILD_TYPE=Release -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=$QT_PATH/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=$QT_PATH/lib/cmake/ -DBUILD_SHARED_LIBS:BOOL=ON ..
make -j $(nproc)
make install -j $(nproc)

## OpenCV
# Install prerequisites
apt-get install -y python-dev python-numpy

# Clone opencv to the target tag
git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv $WORKSPACE/opencv

# Clone opencv_contrib at the target tag, which contains non-free modules (e.g., SURF)
git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv_contrib $WORKSPACE/opencv_contrib

# Compile and install OpenCV with non-free modules (e.g., SURF)
mkdir -p $WORKSPACE/opencv/build
cd $WORKSPACE/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_VTK=ON -D OPENCV_EXTRA_MODULES_PATH=$WORKSPACE/opencv_contrib/modules ..
make -j $(nproc)
make install -j $(nproc)

## PCL
# Install prerequisites
apt-get install -y libboost-all-dev libflann-dev libeigen3-dev

# Clone PCL to the target tag
git clone -b pcl-1.8.0 --single-branch --depth 1 https://github.com/PointCloudLibrary/pcl.git $WORKSPACE/pcl

# Compile and install PCL
mkdir -p $WORKSPACE/pcl/build
cd $WORKSPACE/pcl/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
make install -j $(nproc)


## RTABMap
#Install prerequisites
apt-get install -y libsqlite3-dev

# Clone RTABMap to the target tag
git clone -b 0.12.4 --single-branch --depth 1 https://github.com/introlab/rtabmap $WORKSPACE/rtabmap

# Compile and install RTABMap 0.11.14
mkdir -p $WORKSPACE/rtabmap/build
cd $WORKSPACE/rtabmap/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
make install -j $(nproc)


## Libzbar
apt-get install -y libzbar-dev


## GRPC and Protobuf
#Ref: https://github.com/grpc/grpc/blob/master/INSTALL.md
#Install prerequisites
apt-get install -y autoconf libtool

#Clone latest GRPC and init its submodules
git clone -b v1.3.4 --single-branch --depth 1 --recurse-submodules https://github.com/grpc/grpc $WORKSPACE/grpc

#Compile and install GRPC
cd $WORKSPACE/grpc
make -j $(nproc)
make install -j $(nproc)

# Install protobuf compiled while comopiling GRPC
cd $WORKSPACE/grpc/third_party/protobuf
make install -j $(nproc)


## AprilTags
git clone -b master --single-branch --depth 1 https://github.com/swatbotics/apriltags-cpp.git $WORKSPACE/apriltags

mkdir -p $WORKSPACE/apriltags/build
cd $WORKSPACE/apriltags/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
cp libapriltags.a /usr/local/lib/
mkdir -p /usr/local/include/apriltags
cp ../src/*h /usr/local/include/apriltags/
