//
// Created by bill on 2020/5/2.
//
#ifndef LIDAR3D_LIDAR3D_H
#define LIDAR3D_LIDAR3D_H

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <math.h>
#include <iostream>
#include <atomic>
#include <thread>

#include <pangolin/pangolin.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl-1.11/pcl/point_cloud.h>
#include <pcl-1.11/pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "api.h"

using namespace Eigen;

double get_machine_timestamp_s();

struct Lidar_hps_data{
    int lidar_1_id,lidar_2_id;
    double time_stamp;
    int lidar_data_2[9600][3];
    int lidar_data_1[9600][3];
};

struct Cloud_Filtered_data{
    int size_t;
    double time_stamp;
    int cloud_3d[19200][3];
    int cloud_2d[19200][2];
};

class Lidar3d{
public:
    Lidar3d(){};
    ~Lidar3d() = default;

    double installation_heght = 300 ;

    bool init();

public:
    std::atomic<Lidar_hps_data> hps_data;
    std::atomic<Cloud_Filtered_data> cloud_filtered_data;

private:
    HPS3D_HandleTypeDef handle_lidar3d[DEV_NUM];
    uint8_t connect_number = 0;

    std::thread lidar_thread_;
    bool lidar_thread_func();

    std::thread compute_thread_;
    bool compute_func();

    std::thread display_thread_;
    bool display_func();
};

#endif //LIDAR3D_LIDAR3D_H