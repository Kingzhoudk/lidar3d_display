//
// Created by root on 2020/7/13.
//

#ifndef LIDAR3D_DISPLAY_H
#define LIDAR3D_DISPLAY_H

#include <atomic>
#include <iostream>
#include <thread>
#include <ctime>

struct Test{
    int a;
    double b;
    int c[100];
};

class Distest{
public:
    std::atomic<Test > Mydata;
    Distest(){};
    ~Distest(){}
    bool init();

    std::thread th1;
    void th1_func();

    std::thread th2;
    void th2_func();
};

#endif //LIDAR3D_DISPLAY_H
