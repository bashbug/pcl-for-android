#include <jni.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <iostream>

#include <android/log.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "bashbug.example", __VA_ARGS__))

/*
 * Example:
 * http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd
 * http://pointclouds.org/documentation/tutorials/statistical_outlier.php
 */

extern "C"
JNIEXPORT void JNICALL
Java_com_example_bashbug_example_MainActivity_pclExampleFunc(JNIEnv *env, jobject instance) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width    = 5;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    LOGI("pointcloud has size %lu", cloud->points.size());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh (1.0);
    sor.filter(*cloud_filtered);
}
