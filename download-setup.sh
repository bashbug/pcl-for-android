#!/bin/bash

eigen=3.2.9.tar.bz2
flann=flann-1.8.4-src.zip
boost=boost_1_61_0.tar.bz2

build=build

mkdir ${build}

echo -e "\n\n\033[1;35m###########################################"
echo -e "### EIGEN download and setup            ###"
echo -e "###########################################\033[m\n\n"
[ -f ${eigen} ] || wget http://bitbucket.org/eigen/eigen/get/${eigen} -O ${eigen} || exit 1
mkdir ${build}/eigen
echo "extracting ${eigen} to ${build}/eigen ..."
tar -jxf 3.2.9.tar.bz2 -C ${build}/eigen --strip-components 1 || exit 1

echo -e "\n\n\033[1;35m###########################################"
echo -e "### FLANN download and setup            ###"
echo -e "###########################################\033[m\n\n"
[ -f ${flann} ] || wget http://www.cs.ubc.ca/research/flann/uploads/FLANN/${flann} -O ${flann} || exit 1
echo "extracting ${flann} to ${build}/flann ..."
unzip -o -q flann-1.8.4-src.zip || exit 1
mv flann-1.8.4-src ${build}/flann || exit 1

echo -e "\n\n\033[1;35m###########################################"
echo -e "### BOOST download and setup            ###"
echo -e "###########################################\033[m\n\n"
[ -f ${boost} ] || wget https://sourceforge.net/projects/boost/files/boost/1.61.0/${boost} -O ${boost} || exit 1
mkdir ${build}/boost
echo "extracting ${boost} to ${build}/boost ..."
tar -jxf boost_1_61_0.tar.bz2 -C ${build}/boost --strip-components 1 || exit 1

echo -e "\n\n\033[1;35m###########################################"
echo -e "### PCL download and setup              ###"
echo -e "###########################################\033[m\n\n"
git clone https://github.com/PointCloudLibrary/pcl.git ${build}/pcl
cd ${build}/pcl
git checkout tags/pcl-1.8.0
cd ../..
