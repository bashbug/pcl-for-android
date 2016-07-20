# pcl-for-android

```
$ git clone https://github.com/bashbug/pcl-for-android.git
$ export ANDROID_NDK=PATH_TO_YOUR_LOCAL_ANDROID_NDK_FOLDER
$ ./download-setup.sh
$ ./pcl-build-for-android.sh
```

## download-setup.sh

This script downloads the dependencies:

- EIGEN 3.2.9
- FLANN 1.8.4
- BOOST 1.61.0
- PCL 1.8

## pcl-build-for-android.sh

This script cross compiles FLANN, BOOST and PCL for android. Add the following folders to your Android.mk file:

- eigen
- flann-android
- boost-android
- pcl-android
