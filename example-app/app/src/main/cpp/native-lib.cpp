#include <jni.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include <android/log.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "bashbug.example", __VA_ARGS__))

/*
 * Example: http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd
 */

extern "C"
JNIEXPORT void JNICALL
Java_com_example_bashbug_example_MainActivity_pclExampleFunc(JNIEnv *env, jobject instance) {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    LOGI("pointcloud has size %lu", cloud.points.size());
}
