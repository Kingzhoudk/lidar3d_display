#include "src/lidar3d_data/lidar3d.h"

bool Ugv_System_Error=false;

int main(){
    /*
    // AngleAxisd 旋转向量，沿z轴顺时针旋转
    AngleAxisd rotation_vector(M_PI/2,Vector3d(0,0,1));
    std::cout<<"martrix:"<<rotation_vector.matrix()<<"\n";

    Vector3d v(1,1,0);
    Vector3d v_ro = rotation_vector*v;
    std::cout<<"after:"<<v_ro.transpose()<<"\n";
    */
    Lidar3d lidar;
    lidar.init();

    while(true){
        sleep(10);
    }
}