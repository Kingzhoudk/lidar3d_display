//
// Created by root on 2020/7/13.
//

#include <zconf.h>
#include "display.h"

void Distest::th1_func() {
    int i=0;
    while(true){
        Test ap;
        ap.a=i;
        ap.c[0]=i;
        Mydata.store(ap,std::memory_order_relaxed);
        std::cout<<"write\n";
        i++;
        sleep(1);
    }
}

void Distest::th2_func() {
    int i=0;
    sleep(2);
    while(true){
        Test pa;
        pa=Mydata.load(std::memory_order_relaxed);
        std::cout<<"read,pa.c[0]="<<pa.a;
        sleep(1);
        i++;
    }
}

bool Distest::init() {
    th1=std::thread(&Distest::th1_func,this);
    th2=std::thread(&Distest::th2_func, this);
    return true;
}
