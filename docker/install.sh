apt-get update

set -e

## OpenCV
# Install prerequisites
apt-get install -y git cmake build-essential python-dev python-numpy 

# Clone opencv to the target tag
git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv /root/workspace/opencv

# Clone opencv_contrib at the target tag, which contains non-free modules (e.g., SURF)
git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv_contrib /root/workspace/opencv_contrib

# Compile and install OpenCV with non-free modules (e.g., SURF)
mkdir -p /root/workspace/opencv/build
cd /root/workspace/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/root/workspace/opencv_contrib/modules ..
make -j $(nproc)
make install -j $(nproc)

## Qt5
# ref: https://wiki.qt.io/Building_Qt_5_from_Git
# Install prerequisites
apt-get build-dep -y qt5-default
apt-get install -y libxcb-xinerama0-dev 

# Clone qt5, init repository, checkout to the target tag (only after init-repository has been run), and get all submodules
git clone https://code.qt.io/qt/qt5.git /root/workspace/qt5
cd /root/workspace/qt5
perl init-repository
git checkout tags/v5.8.0
git submodule update

# Generate Makefile for open source project
mkdir -p /root/workspace/qt5/build
cd /root/workspace/qt5/build
../configure -opensource -confirm-license -nomake examples -nomake tests

# Compile and install QT to /usr/local/Qt-%VERSION%
make -j $(nproc) 
make install -j $(nproc)

# Add QT binaries to PATH
echo 'export PATH=$PATH:/usr/local/Qt-5.8.0/bin' >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Qt-5.8.0/lib" >> ~/.bashrc


## VTK
#Ref: http://www.vtk.org/Wiki/VTK/Building/Linux
# Install prerequisites
apt-get install -y libxt-dev

# Clone VTK to the traget tag 
git clone -b v7.1.0 --single-branch --depth 1 https://github.com/Kitware/VTK /root/workspace/VTK

# Compile and install VTK with Qt
mkdir -p /root/workspace/VTK/build
cd /root/workspace/VTK/build
cmake -DCMAKE_BUILD_TYPE=Release -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=/usr/local/Qt-5.8.0/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=/usr/local/Qt-5.8.0/lib/cmake/ -DBUILD_SHARED_LIBS:BOOL=ON ..
make -j $(nproc) 
make install -j $(nproc)

## PCL
# Install prerequisites
apt-get install -y libboost-all-dev libflann-dev libeigen3-dev

# Clone PCL to the target tag
git clone -b pcl-1.8.0 --single-branch --depth 1 https://github.com/PointCloudLibrary/pcl.git /root/workspace/pcl

# Compile and install PCL
mkdir -p /root/workspace/pcl/build
cd /root/workspace/pcl/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
make install -j $(nproc)


## RTABMap
#Install prerequisites
apt-get install -y libsqlite3-dev

# Clone RTABMap to the target tag 
git clone -b 0.11.14 --single-branch --depth 1 https://github.com/introlab/rtabmap /root/workspace/rtabmap

# Compile and install RTABMap 0.11.14
mkdir -p /root/workspace/rtabmap/build
cd /root/workspace/rtabmap/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
make install -j $(nproc)


## BOSSWAVE
#Install prerequisites
apt-get install -y curl

# Install BOSSWAVE 2
curl get.bw2.io/agent | bash

# Clone qtlibbw, which is the C++ binding for BOSSWAVE, and its submodules
git clone --recurse-submodules https://github.com/immesys/qtlibbw /root/workspace/qtlibbw

# Compile and install qtlibbw
mkdir -p /root/workspace/qtlibbw/build
cd /root/workspace/qtlibbw/build
/usr/local/Qt-5.8.0/bin/qmake ../bosswave.pro
make -j $(nproc)
make install -j $(nproc)
cp /root/workspace/qtlibbw/*.h /usr/local/Qt-5.8.0/qml/io/bw2/


## Libmicrohttpd
apt-get install -y libmicrohttpd-dev


## GRPC and Protobuf
#Ref: https://github.com/grpc/grpc/blob/master/INSTALL.md
#Install prerequisites
apt-get install -y autoconf libtool

#Clone latest GRPC and init its submodules
git clone -b $(curl -L http://grpc.io/release) --single-branch --depth 1 --recurse-submodules https://github.com/grpc/grpc /root/workspace/grpc

#Compile and install GRPC
cd /root/workspace/grpc
make -j $(nproc)
make install -j $(nproc)

# Install protobuf compiled while comopiling GRPC
cd /root/workspace/grpc/third_party/protobuf
make install -j $(nproc)


## Remove all files in /root/workspace to reduce docker image size
rm -rf /root/workspace
