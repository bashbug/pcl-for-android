#!/bin/bash

set -e

PROFILE=${1}

if [ ! -f "conan-profiles/${PROFILE}" ] ; then
  echo "${PROFILE} not supported!"
  exit 1
fi

echo -e "\n\n\033[1;35m###########################################"
echo -e "### ANDROID-TOOLCHAIN setup...          ###"
echo -e "###########################################\033[m\n\n"

conan create -pr default conanfiles/android-toolchain bashbug/stable

echo -e "\n\n\033[1;35m###########################################"
echo -e "### FLANN cross-compiling start...      ###"
echo -e "###########################################\033[m\n\n"

conan create -pr conan-profiles/${PROFILE} conanfiles/lz4 bashbug/stable
conan create -pr conan-profiles/${PROFILE} conanfiles/flann bashbug/stable

echo "FLANN cross-compiling finished!"

echo -e "\n\n\033[1;35m###########################################"
echo -e "### BOOST cross-compiling start...      ###"
echo -e "###########################################\033[m\n\n"

conan create -pr conan-profiles/${PROFILE} conanfiles/boost bashbug/stable

echo "BOOST cross-compiling finished!"

echo -e "\n\n\033[1;35mm###########################################"
echo -e "### PCL cross-compiling start...        ###"
echo -e "###########################################\033[m\n\n"

echo -e "\n\n\033[1;32m this will run for a while... time to drink a\n"
echo -e "   ( ( "
echo -e "    ) ) "
echo -e "  ........ "
echo -e "  |      |] "
echo -e "  \      /  "
echo -e "   '----' \033[m\n\n"

conan create -pr conan-profiles/${PROFILE} conanfiles/pcl bashbug/stable

cp conan-profiles/${PROFILE} ~/.conan/profiles

echo "PCL cross-compiling finished!"
