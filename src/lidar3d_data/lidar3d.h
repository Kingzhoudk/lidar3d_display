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

#include "api.h"

struct Lidar3d_data{
    double id;
    int distance[9600][3];
    double lidar_left_distance;
    double lidar_right_distance;
};

int lidar3d_plan(Lidar3d_data data,int width,int height,double Ob_distance);

class Lidar3d{
public:
    Lidar3d(){};
    ~Lidar3d() = default;

    bool init(){
        std::cout << "lidar read thread begin" << std::endl;
        lidar_thread_ = std::thread(&Lidar3d::lidar_thread_func, this);
        return true;
    }

public:
    HPS3D_HandleTypeDef handle_lidar3d[DEV_NUM];
    uint8_t connect_number = 0;

    Lidar3d_data mydata;

private:
    std::thread lidar_thread_;
    bool lidar_thread_func();

private:
    std::thread display_thread_;
    bool display_thread_func();
};


#endif //LIDAR3D_LIDAR3D_H