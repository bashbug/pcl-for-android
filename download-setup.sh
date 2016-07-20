#!/bin/bash

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### EIGEN download and setup            ###"
echo -e "###########################################\033[m"
echo
echo
wget http://bitbucket.org/eigen/eigen/get/3.2.9.tar.bz2
tar -jxf 3.2.9.tar.bz2
mkdir eigen
mv eigen-eigen-*/* eigen
rm -rf 3.2.9.tar.bz2 eigen-eigen-*

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### FLANN download and setup            ###"
echo -e "###########################################\033[m"
echo
echo
wget http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip
unzip flann-1.8.4-src.zip
mkdir flann
mv flann-1.8.4-src/* flann
rm -rf flann-1.8.4-src.zip flann-1.8.4-src

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### BOOST download and setup            ###"
echo -e "###########################################\033[m"
echo
echo
wget https://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.bz2
tar -jxf boost_1_61_0.tar.bz2
mkdir boost
mv boost_1_61_0/* boost
rm -rf boost_1_61_0.tar.bz2 boost_1_61_0

echo
echo
echo -e "\033[1;35m###########################################"
echo -e "### PCL download and setup              ###"
echo -e "###########################################\033[m"
echo
echo

git clone https://github.com/PointCloudLibrary/pcl.git
