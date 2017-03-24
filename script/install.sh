#!/usr/bin/env bash

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
git clone https://code.qt.io/qt/qt5.git $HOME/workspace/qt5
cd $HOME/workspace/qt5
perl init-repository
git checkout tags/v5.8.0
git submodule update

# Generate Makefile for open source project
mkdir -p $HOME/workspace/qt5/build
cd $HOME/workspace/qt5/build
../configure -opensource -confirm-license -nomake examples -nomake tests

# Compile and install QT to /usr/loca/Qt-%VERSION%
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
git clone -b v7.1.0 --single-branch --depth 1 https://github.com/Kitware/VTK $HOME/workspace/VTK

# Compile and install VTK with Qt
mkdir -p $HOME/workspace/VTK/build
cd $HOME/workspace/VTK/build
cmake -DCMAKE_BUILD_TYPE=Release -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=$QT_PATH/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=$QT_PATH/lib/cmake/ -DBUILD_SHARED_LIBS:BOOL=ON ..
make -j $(nproc)
make install -j $(nproc)

## OpenCV
# Install prerequisites
apt-get install -y python-dev python-numpy

# Clone opencv to the target tag
git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv $HOME/workspace/opencv

# Clone opencv_contrib at the target tag, which contains non-free modules (e.g., SURF)
git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv_contrib $HOME/workspace/opencv_contrib

# Compile and install OpenCV with non-free modules (e.g., SURF)
mkdir -p $HOME/workspace/opencv/build
cd $HOME/workspace/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_VTK=ON OPENCV_EXTRA_MODULES_PATH=$HOME/workspace/opencv_contrib/modules ..
make -j $(nproc)
make install -j $(nproc)

## PCL
# Install prerequisites
apt-get install -y libboost-all-dev libflann-dev libeigen3-dev

# Clone PCL to the target tag
git clone -b pcl-1.8.0 --single-branch --depth 1 https://github.com/PointCloudLibrary/pcl.git $HOME/workspace/pcl

# Compile and install PCL
mkdir -p $HOME/workspace/pcl/build
cd $HOME/workspace/pcl/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
make install -j $(nproc)


## RTABMap
#Install prerequisites
apt-get install -y libsqlite3-dev

# Clone RTABMap to the target tag
git clone -b 0.11.14 --single-branch --depth 1 https://github.com/introlab/rtabmap $HOME/workspace/rtabmap

# Compile and install RTABMap 0.11.14
mkdir -p $HOME/workspace/rtabmap/build
cd $HOME/workspace/rtabmap/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
make install -j $(nproc)


## BOSSWAVE
#Install prerequisites
apt-get install -y curl

# Install BOSSWAVE 2
curl get.bw2.io/agent | bash

# Clone qtlibbw, which is the C++ binding for BOSSWAVE, and its submodules
git clone --recurse-submodules https://github.com/immesys/qtlibbw $HOME/workspace/qtlibbw

# Compile and install qtlibbw
mkdir -p $HOME/workspace/qtlibbw/build
cd $HOME/workspace/qtlibbw/build
$QT_PATH/bin/qmake ../bosswave.pro
make -j $(nproc)
make install -j $(nproc)
cp $HOME/workspace/qtlibbw/*.h $QT_PATH/qml/io/bw2/


## Libmicrohttpd
apt-get install -y libmicrohttpd-dev


## GRPC and Protobuf
#Ref: https://github.com/grpc/grpc/blob/master/INSTALL.md
#Install prerequisites
apt-get install -y autoconf libtool

#Clone latest GRPC and init its submodules
git clone -b v1.2.0 --single-branch --depth 1 --recurse-submodules https://github.com/grpc/grpc $HOME/workspace/grpc

#Compile and install GRPC
cd $HOME/workspace/grpc
make -j $(nproc)
make install -j $(nproc)

# Install protobuf compiled while comopiling GRPC
cd $HOME/workspace/grpc/third_party/protobuf
make install -j $(nproc)
