mkdir -p downloads
cd downloads

wget -nc http://www.cs.berkeley.edu/~kaifei/download/bw2android-0.1-all.jar -P ../app/libs

wget -nc https://sourceforge.net/projects/opencvlibrary/files/opencv-android/2.4.11/OpenCV-2.4.11-android-sdk.zip/download -O OpenCV-2.4.11-android-sdk.zip
if [ ! -d OpenCV-android-sdk ]; then
    unzip OpenCV-2.4.11-android-sdk.zip
fi

mkdir -p ../app/src/main/3rdparty
rsync -avz ./OpenCV-android-sdk/sdk/native/3rdparty/ ../app/src/main/3rdparty

mkdir -p ../app/src/main/jni/include
rsync -avz ./OpenCV-android-sdk/sdk/native/jni/include/ ../app/src/main/jni/include

mkdir -p ../app/src/main/jniLibs
rsync -avz ./OpenCV-android-sdk/sdk/native/libs/ ../app/src/main/jniLibs

mkdir -p ../openCVLibrary2411
cp ./OpenCV-android-sdk/sdk/java/lint.xml ../openCVLibrary2411

mkdir -p ../openCVLibrary2411/src/main/java
rsync -avz --exclude 'org/opencv/engine/' ./OpenCV-android-sdk/sdk/java/src/ ../openCVLibrary2411/src/main/java

mkdir -p ../openCVLibrary2411/src/main/aidl/org/opencv/engine
rsync -avz ./OpenCV-android-sdk/sdk/java/src/org/opencv/engine/ ../openCVLibrary2411/src/main/aidl/org/opencv/engine

mkdir -p ../openCVLibrary2411/src/main/res
rsync -avz ./OpenCV-android-sdk/sdk/java/res/ ../openCVLibrary2411/src/main/res

cd ..
