# install basic stuff
apt-get update
apt-get install -y git

# install pcl 1.8
apt-get install -y build-essential
apt-get install -y libboost-all-dev
apt-get install -y libflann-dev
apt-get install -y libopenni-dev
apt-get install -y cmake
apt-get install -y libvtk6-dev
apt-get install -y libeigen3-dev
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/PointCloudLibrary/pcl.git
mkdir -p pcl/build
cd pcl/build
git checkout pcl-1.8.0
ln -s /usr/bin/vtk6 /usr/bin/vtk # this symlink is missing
ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.x86_64-linux-gnu.so /usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so # this symlink is missing
ln -s /usr/lib/x86_64-linux-gnu/libproj.so.9 /usr/lib/x86_64-linux-gnu/libproj.so # this symlink is missing
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make install

# install QT5.5 (latest 5.7)
apt-get install -y qt5-default

# install Nvidia driver 367
sudo add-apt-repository ppa:graphics-drivers/ppa
apt-get install -y nvidia-367

# install cuda 7.5
echo "deb http://cz.archive.ubuntu.com/ubuntu xenial main multiverse" >> /etc/apt/sources.list
apt-get update
apt-get install -y nvidia-cuda-toolkit

# install opencv 3.1.0 with extra modules (SURF, etc.)
cd ~/workspace/
git clone https://github.com/Itseez/opencv.git
cd opencv
git checkout 3.1.0
cd ~/workspace/
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv_contrib
git checkout 3.1.0
mkdir -p ~/workspace/opencv/build
cd ~/workspace/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D WITH_CUBLAS=1 -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D OPENCV_EXTRA_MODULES_PATH="../../opencv_contrib/modules" ..
make -j8
make install

# install RTAB-Map
apt-get install -y libsqlite3-dev
cd ~/workspace/
git clone https://github.com/introlab/rtabmap.git
mkdir -p rtabmap/build
cd rtabmap/build
git checkout 0.11.8
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make install

# install CellMate server 
apt-get install -y libmicrohttpd-dev
cd ~/workspace/
git clone https://github.com/kaifeichen/CellMate.git
cd CellMate/server/build
cmake ..
make -j8
make install
