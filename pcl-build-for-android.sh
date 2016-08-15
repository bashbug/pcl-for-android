#!/bin/bash

# specify the arm as abi, the api level for android kitkat as used by
# google tango and with gnustl_static the c++ support
# for more information look into the android.toolchain.cmake file
export ANDROID_ABI="armeabi armeabi-v7a with NEON"
export ANDROID_NATIVE_API_LEVEL=android-19
export ANDROID_STL=gnustl_static
export ANDROID_STL_FORCE_FEATURES=ON

export CFLAGS="-pipe -w"
export CXXFLAGS=${CFLAGS}

[[ -z "${jobs}" ]] && jobs=$(grep -cP '^processor' /proc/cpuinfo)

echo -e "\n\n\033[1;32mCompiling with ${jobs} jobs ...\033[m"

# check that ANDROID_NDK points to a android ndk folder
if [[ ! -d ${ANDROID_NDK} ]]
then
  echo -e "\e[1;31m[ERROR] ANDROID_NDK is not found! Set it as follows: export ANDROID_NDK=PATH_TO_YOUR_LOCAL_ANDROID_NDK_FOLDER\033[m"
  exit 1
fi

cd build

ROOT=${PWD}
ANDROIDTOOLCHAIN=${ROOT}/../android.toolchain.cmake

EIGEN_INCLUDE_DIR=${ROOT}/eigen # eigen needs no cross-compiling
FLANN_ROOT=${ROOT}/flann
BOOST_ROOT=${ROOT}/boost
PCL_ROOT=${ROOT}/pcl

echo -e "\n\n\033[1;35m###########################################"
echo -e "### FLANN cross-compiling start...      ###"
echo -e "###########################################\033[m\n\n"

cd ${FLANN_ROOT}

cmake . -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$ANDROIDTOOLCHAIN \
  -DBUILD_EXAMPLES:BOOL=OFF \
  -DBUILD_PYTHON_BINDINGS:BOOL=OFF \
  -DBUILD_MATLAB_BINDINGS:BOOL=OFF

make -j${jobs}

cd ..
echo "FLANN cross-compiling finished!"

### copy build lib files and headers to correct destination

rm -rf ${ROOT}/flann-android
mkdir -p ${ROOT}/flann-android/lib
mkdir -p ${ROOT}/flann-android/include

mv ${ROOT}/flann/lib/* ${ROOT}/flann-android/lib
cp -rf ${ROOT}/flann/src/cpp/flann ${ROOT}/flann-android/include

FLANN_LIBRARY=${ROOT}/flann-android/lib
FLANN_INCLUDE_DIR=${ROOT}/flann-android/include

echo -e "\n\n\033[1;35m###########################################"
echo -e "### BOOST cross-compiling start...      ###"
echo -e "###########################################\033[m\n\n"

# move boost make file for cross-compiling
cp ${ROOT}/../CMakeLists.txt ${BOOST_ROOT}

cd ${BOOST_ROOT}

cmake . -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$ANDROIDTOOLCHAIN \
  -DBUILD_SHARED_LIBS:BOOL=OFF

make -j${jobs}

cd ..
echo "BOOST cross-compiling finished!"

### copy build lib files and headers to correct destination

rm -rf ${ROOT}/boost-android
mkdir -p ${ROOT}/boost-android/lib
mkdir -p ${ROOT}/boost-android/include

mv ${ROOT}/boost/libboost*.a ${ROOT}/boost-android/lib/
cp -rf ${BOOST_ROOT}/boost/ ${ROOT}/boost-android/include

export BOOST_ROOT=${ROOT}/boost-android
Boost_LIBRARIES=${ROOT}/boost-android/lib
Boost_INCLUDE_DIRS=${ROOT}/boost-android/include

echo -e "\n\n\033[1;35mm###########################################"
echo -e "### PCL cross-compiling start...        ###"
echo -e "###########################################\033[m\n\n"

cd ${PCL_ROOT}

[ -f Makefile ] && make clean

rm -rf ${ROOT}/pcl/CMakeCache.txt ${ROOT}/pcl/CMakeFiles

rm -rf ${ROOT}/pcl-android
mkdir ${ROOT}/pcl-android

function cmake_pcl {
  cmake . -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INSTALL_PREFIX=${ROOT}/pcl-android \
    -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$ANDROIDTOOLCHAIN \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -DPCL_SHARED_LIBS:BOOL=OFF \
    -DBUILD_visualization:BOOL=OFF \
    -DBUILD_examples:BOOL=OFF \
    -DEIGEN_INCLUDE_DIR:PATH=${EIGEN_INCLUDE_DIR} \
    -DFLANN_INCLUDE_DIR:PATH=${FLANN_INCLUDE_DIR} \
    -DFLANN_LIBRARY:FILEPATH=${FLANN_LIBRARY}/libflann_cpp_s.a \
    -DBOOST_ROOT:PATH=${BOOST_ROOT} \
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
}

### pcl cmake files do not set library paths properly. Therefore cmake has to be run twice.
cmake_pcl
cmake_pcl

echo -e "\n\n\033[1;32m make -j$jobs\033[m\n"
echo -e "\n\n\033[1;32m this will run for a while... time to drink a\n"
echo -e "   ( ( "
echo -e "    ) ) "
echo -e "  ........ "
echo -e "  |      |] "
echo -e "  \      /  "
echo -e "   '----' \033[m\n\n"

make -j${jobs}


echo -e "\n\n\033[1;35m make install\033[m\n\n"
make install

cd ..
echo "PCL cross-compiling finished!"
