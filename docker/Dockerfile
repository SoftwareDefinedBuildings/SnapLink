FROM ubuntu:latest
MAINTAINER Kaifei Chen "kaifei@berkeley.edu"

RUN apt-get update

## OpenCV
# Install prerequisites
RUN apt-get install -y git cmake build-essential python-dev python-numpy 

# Clone opencv to the target tag
RUN git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv /root/workspace/opencv
RUN ls -alh /root/workspace/opencv

# Clone opencv_contrib at the target tag, which contains non-free modules (e.g., SURF)
RUN git clone -b 3.2.0 --single-branch --depth 1 https://github.com/opencv/opencv_contrib /root/workspace/opencv_contrib

# Compile and install OpenCV with non-free modules (e.g., SURF)
RUN mkdir -p /root/workspace/opencv/build
WORKDIR /root/workspace/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/root/workspace/opencv_contrib/modules ..
RUN make -j $(nproc)
RUN make install -j $(nproc)

## Qt5
# ref: https://wiki.qt.io/Building_Qt_5_from_Git
# Install prerequisites
RUN apt-get build-dep -y qt5-default
RUN apt-get install -y libxcb-xinerama0-dev 

# Clone qt5, init repository, checkout to the target tag (only after init-repository has been run), and get all submodules
RUN git clone git://code.qt.io/qt/qt5.git /root/workspace/qt5
WORKDIR /root/workspace/qt5
RUN perl init-repository
RUN git checkout tags/v5.8.0
RUN git submodule update

# Generate Makefile for open source project
RUN mkdir -p /root/workspace/qt5/build
WORKDIR /root/workspace/qt5/build
RUN ../configure -opensource -confirm-license -nomake examples -nomake tests

# Compile and install QT to /usr/local/Qt-%VERSION%
RUN make -j $(nproc) 
RUN make install -j $(nproc)

# Add QT binaries to PATH
RUN echo 'export PATH=/usr/local/Qt-5.8.0/bin:$PATH' >> ~/.bashrc 


## VTK
#Ref: http://www.vtk.org/Wiki/VTK/Building/Linux
# Install prerequisites
RUN apt-get install -y libxt-dev

# Clone VTK to the traget tag 
RUN git clone -b v7.1.0 --single-branch --depth 1 https://github.com/Kitware/VTK /root/workspace/VTK

# Compile and install VTK with Qt
RUN mkdir -p /root/workspace/VTK/build
WORKDIR /root/workspace/VTK/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=/usr/local/Qt-5.8.0/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=/usr/local/Qt-5.8.0/lib/cmake/ -DBUILD_SHARED_LIBS:BOOL=ON ..
RUN make -j $(nproc) 
RUN make install -j $(nproc)

## PCL
# Install prerequisites
RUN apt-get install -y libboost-all-dev libflann-dev libeigen3-dev

# Clone PCL to the target tag
RUN git clone -b pcl-1.8.0 --single-branch --depth 1 https://github.com/PointCloudLibrary/pcl.git /root/workspace/pcl

# Compile and install PCL
RUN mkdir -p /root/workspace/pcl/build
WORKDIR /root/workspace/pcl/build
RUN cmake -DCMAKE_BUILD_TYPE=Release ..
RUN make -j $(nproc)
RUN make install -j $(nproc)


## RTABMap
#Install prerequisites
RUN apt-get install -y libsqlite3-dev

# Clone RTABMap to the target tag 
RUN git clone -b 0.11.14 --single-branch --depth 1 https://github.com/introlab/rtabmap /root/workspace/rtabmap

# Compile and install RTABMap 0.11.14
RUN mkdir -p /root/workspace/rtabmap/build
WORKDIR /root/workspace/rtabmap/build
RUN cmake -DCMAKE_BUILD_TYPE=Release ..
RUN make -j $(nproc)
RUN make install -j $(nproc)


## BOSSWAVE
#Install prerequisites
RUN apt-get install -y curl

# Install BOSSWAVE 2
RUN curl get.bw2.io/agent | bash

# Clone qtlibbw, which is the C++ binding for BOSSWAVE, and its submodules
RUN git clone --recurse-submodules https://github.com/immesys/qtlibbw /root/workspace/qtlibbw

# Compile and install qtlibbw
RUN mkdir -p /root/workspace/qtlibbw/build
WORKDIR /root/workspace/qtlibbw/build
RUN /usr/local/Qt-5.8.0/bin/qmake ../bosswave.pro
RUN make -j $(nproc)
RUN make install -j $(nproc)
RUN cp /root/workspace/qtlibbw/*.h /usr/local/Qt-5.8.0/qml/io/bw2/


## Libmicrohttpd
RUN apt-get install -y libmicrohttpd-dev

## GRPC and Protobuf
#Ref: https://github.com/grpc/grpc/blob/master/INSTALL.md
#Install prerequisites
RUN apt-get install -y autoconf libtool

#Clone latest GRPC and init its submodules
RUN git clone -b $(curl -L http://grpc.io/release) --single-branch --depth 1 --recurse-submodules https://github.com/grpc/grpc /root/workspace/grpc

#Compile and install GRPC
WORKDIR /root/workspace/grpc
RUN make -j $(nproc)
RUN make install -j $(nproc)

# Install protobuf compiled while comopiling GRPC
WORKDIR /root/workspace/grpc/third_party/protobuf
RUN make install -j $(nproc)


# make sure all cloned files are not inclued in the image
VOLUME /root/workspace
