#!/bin/bash

# specify the arm as abi, the api level for android kitkat as used by
# google tango and with gnustl_static the c++ support
# for more information look into the android.toolchain.cmake file
export ANDROID_ABI="armeabi armeabi-v7a with NEON x86_64"
export ANDROID_NATIVE_API_LEVEL=android-19
export ANDROID_STL=gnustl_static
export ANDROID_STL_FORCE_FEATURES=ON

# check that ANDROID_NDK points to a android ndk folder
if [[ ! -d ${ANDROID_NDK} ]]
then
  echo -e "\e[1;31m[ERROR] ANDROID_NDK is not found! Set it as follows: export ANDROID_NDK=PATH_TO_YOUR_LOCAL_ANDROID_NDK_FOLDER\033[m"
  exit 1
fi

ROOT=${PWD}
ANDROIDTOOLCHAIN=${ROOT}/android.toolchain.cmake

EIGEN_INCLUDE_DIR=${ROOT}/eigen # eigen needs no cross-compiling
FLANN_ROOT=${ROOT}/flann
BOOST_ROOT=${ROOT}/boost
PCL_ROOT=${ROOT}/pcl

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### FLANN cross-compiling start...      ###"
echo -e "###########################################\033[m"
echo
echo

cd ${FLANN_ROOT}
cmake . -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$ANDROIDTOOLCHAIN \
  -DBUILD_EXAMPLES:BOOL=OFF \
  -DBUILD_PYTHON_BINDINGS:BOOL=OFF \
  -DBUILD_MATLAB_BINDINGS:BOOL=OFF

make -j4

cd ..
echo "FLANN cross-compiling stop!"

### copy build lib files and headers to correct destination

rm -rf ${ROOT}/flann-android
mkdir ${ROOT}/flann-android
mkdir ${ROOT}/flann-android/lib
mkdir ${ROOT}/flann-android/include

mv ${ROOT}/flann/lib/* ${ROOT}/flann-android/lib
cp -rf ${ROOT}/flann/src/cpp/flann ${ROOT}/flann-android/include

FLANN_LIBRARY=${ROOT}/flann-android/lib
FLANN_INCLUDE_DIR=${ROOT}/flann-android/include

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### BOOST cross-compiling start...      ###"
echo -e "###########################################\033[m"
echo
echo

# move boost make file for cross-compiling
mv ${ROOT}/CMakeLists.txt ${BOOST_ROOT}

cd ${BOOST_ROOT}
cmake . -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$ANDROIDTOOLCHAIN \
  -DBUILD_SHARED_LIBS:BOOL=OFF

make -j4

cd ..
echo "BOOST cross-compiling stop!"

### copy build lib files and headers to correct destination

rm -rf ${ROOT}/boost-android
mkdir ${ROOT}/boost-android
mkdir ${ROOT}/boost-android/lib

mv ${ROOT}/boost/libboost*.a ${ROOT}/boost-android/lib/
cp -rf ${BOOST_ROOT}/boost/ ${ROOT}/boost-android/

BOOST_ROOT=${ROOT}/boost-android
Boost_LIBRARIES=${ROOT}/boost-android/lib
Boost_INCLUDE_DIRS=${ROOT}/boost

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### PCL cross-compiling start...        ###"
echo -e "###########################################\033[m"
echo
echo

cd ${PCL_ROOT}

rm -rf ${ROOT}/pcl/CMakeCache.txt ${ROOT}/pcl/CMakeFiles

export BOOST_ROOT=${ROOT}/boost-android

cmake . -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$ANDROIDTOOLCHAIN \
  -DBUILD_SHARED_LIBS:BOOL=OFF \
  -DPCL_SHARED_LIBS:BOOL=OFF \
  -DBUILD_visualization:BOOL=OFF \
  -DBUILD_examples:BOOL=OFF \
  -DEIGEN_INCLUDE_DIR:PATH=${EIGEN_INCLUDE_DIR} \
  -DFLANN_INCLUDE_DIR:PATH=${FLANN_INCLUDE_DIR} \
  -DFLANN_LIBRARY:FILEPATH=${FLANN_LIBRARY}/libflann_cpp_s.a \
  -DBOOST_ROOT:PATH=${ROOT}/boost-android \
  -DBoost_INCLUDE_DIR:PATH=${Boost_INCLUDE_DIRS} \
  -DWITH_VTK:BOOL=OFF \
  -DWITH_QHULL:BOOL=OFF \
  -DWITH_PCAP:BOOL=OFF \
  -DWITH_PNG:BOOL=OFF \
  -DWITH_OPENGL:BOOL=OFF \
  -DWITH_LIBUSB:BOOL=OFF \
  -DBoost_DATE_TIME_LIBRARY=${Boost_LIBRARIES}/libboost_date_time.a \
  -DBoost_DATE_TIME_LIBRARY_DEBUG=${Boost_LIBRARIES}/libboost_date_time.a \
  -DBoost_DATE_TIME_LIBRARY_RELEASE=${Boost_LIBRARIES}/libboost_date_time.a \
  -DBoost_FILESYSTEM_LIBRARY=${Boost_LIBRARIES}/libboost_filesystem.a \
  -DBoost_FILESYSTEM_LIBRARY_DEBUG=${Boost_LIBRARIES}/libboost_filesystem.a \
  -DBoost_FILESYSTEM_LIBRARY_RELEASE=${Boost_LIBRARIES}/libboost_filesystem.a \
  -DBoost_IOSTREAMS_LIBRARY=${Boost_LIBRARIES}/libboost_iostreams.a \
  -DBoost_IOSTREAMS_LIBRARY_DEBUG=${Boost_LIBRARIES}/libboost_iostreams.a \
  -DBoost_IOSTREAMS_LIBRARY_RELEASE=${Boost_LIBRARIES}/libboost_iostreams.a \
  -DBoost_SYSTEM_LIBRARY=${Boost_LIBRARIES}/libboost_system.a \
  -DBoost_SYSTEM_LIBRARY_DEBUG=${Boost_LIBRARIES}/libboost_system.a \
  -DBoost_SYSTEM_LIBRARY_RELEASE=${Boost_LIBRARIES}/libboost_system.a \
  -DBoost_THREAD_LIBRARY=${Boost_LIBRARIES}/libboost_thread.a \
  -DBoost_THREAD_LIBRARY_DEBUGBoost_INCLUDE_DIRS=${Boost_LIBRARIES}/libboost_thread.a \
  -DBoost_THREAD_LIBRARY_RELEASE=${Boost_LIBRARIES}/libboost_thread.a \
  -DBoost_LIBRARY_DIRS=${Boost_LIBRARIES}

make -j4

cd ..
echo "PCL cross-compiling stop!"

### copy build lib files and headers to correct destination

rm -rf ${ROOT}/pcl-android
mkdir ${ROOT}/pcl-android
mkdir ${ROOT}/pcl-android/lib
mkdir ${ROOT}/pcl-android/include
mkdir ${ROOT}/pcl-android/include/pcl

PCL_ANDROID_ROOT=${ROOT}/pcl-android

mv ${ROOT}/pcl/lib/* ${ROOT}/pcl-android/lib

if [ $(ls -1 ${PCL_ROOT}/include/pcl/ | wc -l) -eq 1 ]
then
  echo -e "\e[1;31m[FIX] include header structure from"
  echo -e "      include/pcl/module/include/pcl/"
  echo -e "to"
  echo -e "      include/pcl/module\e[m"
  for DIR in ${PCL_ROOT}/*
  do
    if [ -d "${DIR}/include" ]
    then
      cp -rf ${DIR}/include/pcl/* ${PCL_ANDROID_ROOT}/include/pcl
    fi
  done
  cp ${PCL_ROOT}/include/pcl/pcl_config.h ${PCL_ANDROID_ROOT}/include/pcl
else
  cp -rf ${PCL_ROOT}/include/pcl/* ${PCL_ANDROID_ROOT}/include/pcl
fi
