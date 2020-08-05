//
// Created by zhoudk on 2020/5/2.
//

#include "lidar3d.h"

double get_machine_timestamp_s() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> tp = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
//    std::cout.setf(std::ios::fixed,std::ios::floatfield);
//    std::cout<<"now_time:"<<time_stamp<<"\n";
    return (static_cast<double>(timestamp)) / 1000000.0;
}

bool Lidar3d::init() {
    std::cout << "HPS3D lidar read thread begin" << std::endl;
    lidar_thread_ = std::thread(&Lidar3d::lidar_thread_func, this);
    sleep(5);

    compute_thread_ = std::thread(&Lidar3d::compute_func, this);

    //barrier_thread_ = std::thread(&Lidar3d::barrier_func,this);

    display_thread_ = std::thread(&Lidar3d::display_func,this);
    return true;
}

bool Lidar3d::lidar_thread_func() {
    uint32_t a = 0;
    /*Init Device*/
    do{
        // 设置测量包类型
        HPS3D_SetMeasurePacketType(ROI_DATA_PACKET);

        connect_number = HPS3D_AutoConnectAndInitConfigDevice(handle_lidar3d);
        printf("connect_number = %d\n",connect_number);
        if(connect_number == 0){
            printf("error,connect_number = %d\n",connect_number);
            return false;
        }
        // 开启边缘滤波
        HPS3D_SetEdgeDetectionEnable (true);
        HPS3D_GetEdgeDetectionEnable ();
        HPS3D_SetEdgeDetectionValue (1000);
        HPS3D_GetEdgeDetectionValue();
        // 获取点云数据
        HPS3D_SetPointCloudEn(true);
        HPS3D_GetPointCloudEn();

        for(int i = 0;i<connect_number;i++){
            //设定偏移，距离差0cm
            HPS3D_SetDistanceOffset(&handle_lidar3d[i], -0);
            HPS3D_SetOpticalEnable(&handle_lidar3d[i],true);
            // 设置数据类型
            handle_lidar3d[i].RetPacketType=FULL_ROI_PACKET;
            // 单次测量
            handle_lidar3d[i].RunMode = RUN_SINGLE_SHOT;
        }
        a = 1;
    }while(a==0);

    Lidar_hps_data my_data;
    // 15 or 10 id
    if(HPS3D_GetMultiCameraCode(&handle_lidar3d[0])==15){
        my_data.lidar_1_id=0;
        my_data.lidar_2_id=1;
    }
    else{
        my_data.lidar_1_id=1;
        my_data.lidar_2_id=0;
    }

    std::ofstream OutFile("0803roadmap.txt", std::ios::trunc);
    while(a==1) {
        if(HPS3D_SingleMeasurement(&handle_lidar3d[0]) != RET_OK || HPS3D_SingleMeasurement(&handle_lidar3d[1]) != RET_OK){
            std::cout<<"error!";
        }
        else{
            std::cout<<"1111!";
        }
        for (int i = 0; i < 9600; i++) {
            my_data.lidar_data_1[i][0] = handle_lidar3d[my_data.lidar_1_id].MeasureData.point_cloud_data->point_data[i].x;
            my_data.lidar_data_1[i][1] = handle_lidar3d[my_data.lidar_1_id].MeasureData.point_cloud_data->point_data[i].y;
            my_data.lidar_data_1[i][2] = handle_lidar3d[my_data.lidar_1_id].MeasureData.point_cloud_data->point_data[i].z;
            std::ofstream OutFile("0803roadmap.txt", std::ios::app);
            OutFile <<my_data.lidar_data_1[i][0]<< ',' << my_data.lidar_data_1[i][1] << ',' << my_data.lidar_data_1[i][2]<<"\n";
            OutFile.close();

            my_data.lidar_data_2[i][0] = handle_lidar3d[my_data.lidar_2_id].MeasureData.point_cloud_data->point_data[i].x;
            my_data.lidar_data_2[i][1] = handle_lidar3d[my_data.lidar_2_id].MeasureData.point_cloud_data->point_data[i].y;
            my_data.lidar_data_2[i][2] = handle_lidar3d[my_data.lidar_2_id].MeasureData.point_cloud_data->point_data[i].z;

        }
        std::ofstream OutFile("0803roadmap.txt", std::ios::app);
        OutFile <<"\n\n";
        OutFile.close();
        my_data.time_stamp = get_machine_timestamp_s();
        hps_data.store(my_data, std::memory_order_relaxed);
    }
    return true;
}

bool Lidar3d::compute_func() {
    Lidar_hps_data my_hps;
    Cloud_Filtered_data my_cloud;
    Barrier_data my_barrier;
    // AngleAxisd 旋转向量，沿y轴逆时针旋转38度
    AngleAxisd rotation_vector_1(0.6632251,Vector3d(0,-1,0));
    // AngleAxisd 旋转向量，沿y轴顺时针旋转38度
    AngleAxisd rotation_vector_2(0.6632251,Vector3d(0,1,0));

    Vector3d point_v,point_v_ro,point_cloud;
    std::vector<Vector3d > cloud_;
    int a=1;

    while(a){

        double time_stamp = get_machine_timestamp_s();
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout << "now_time:" << time_stamp << "\n";

        my_hps=hps_data.load(std::memory_order_relaxed);
        int cout=0;
        for(int i=0;i<9600;i++){
            point_v.x()=my_hps.lidar_data_1[i][0];
            point_v.y()=my_hps.lidar_data_1[i][1];
            point_v.z()=my_hps.lidar_data_1[i][2];
            point_v_ro=rotation_vector_1*point_v;
            // point_v.y()<installation_heght 去除地面
            if(point_v.z()>=250 && point_v.z()<40000 && point_v.y()< installation_heght) {
                // 3d
                my_cloud.cloud_3d[cout][0]=point_v_ro.x();
                my_cloud.cloud_3d[cout][1]=point_v_ro.y();
                my_cloud.cloud_3d[cout][2]=point_v_ro.z();
                //
                my_cloud.cloud_2d[cout][0]=point_v_ro.x();
                my_cloud.cloud_2d[cout][1]=point_v_ro.z();
                cout++;
            }
            point_v.x()=my_hps.lidar_data_2[i][0];
            point_v.y()=my_hps.lidar_data_2[i][1];
            point_v.z()=my_hps.lidar_data_2[i][2];
            point_v_ro=rotation_vector_2*point_v;
            // point_v.y()<installation_heght 去除地面
            if(point_v.z()>=250 && point_v.z()<40000 && point_v.y()< installation_heght) {
                // 3d
                my_cloud.cloud_3d[cout][0]=point_v_ro.x();
                my_cloud.cloud_3d[cout][1]=point_v_ro.y();
                my_cloud.cloud_3d[cout][2]=point_v_ro.z();
                //
                my_cloud.cloud_2d[cout][0]=point_v_ro.x();
                my_cloud.cloud_2d[cout][1]=point_v_ro.z();
                cout++;
            }
        }
        my_cloud.size_t=cout;
        my_cloud.time_stamp = my_hps.time_stamp;
        cloud_filtered_data.store(my_cloud, std::memory_order_relaxed);

        // compute barrier
        int distance[8]={9999,9999,9999,9999,9999,9999,9999,9999},dis_cout[8]={0};
        my_cloud=cloud_filtered_data.load(std::memory_order_relaxed);
        for(int i=0;i<my_cloud.size_t;i++){
            if( my_cloud.cloud_2d[i][0] > -1000&&  my_cloud.cloud_2d[i][0] <= -750){
                if(distance[0]>my_cloud.cloud_2d[i][1])
                    distance[0]=my_cloud.cloud_2d[i][1];
                dis_cout[0]++;
            }
            else if( my_cloud.cloud_2d[i][0] > -750 &&  my_cloud.cloud_2d[i][0] <= -500){
                if(distance[1]>my_cloud.cloud_2d[i][1])
                    distance[1]=my_cloud.cloud_2d[i][1];
                dis_cout[1]++;
            }
            else if( my_cloud.cloud_2d[i][0] > -500 &&  my_cloud.cloud_2d[i][0] <= -250){
                if(distance[2]>my_cloud.cloud_2d[i][1])
                    distance[2]=my_cloud.cloud_2d[i][1];
                dis_cout[2]++;
            }
            else if( my_cloud.cloud_2d[i][0] > -250 &&  my_cloud.cloud_2d[i][0] <= 0){
                if(distance[3]>my_cloud.cloud_2d[i][1])
                    distance[3]=my_cloud.cloud_2d[i][1];
                dis_cout[3]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 0    &&  my_cloud.cloud_2d[i][0] <= 250){
                if(distance[4]>my_cloud.cloud_2d[i][1])
                    distance[4]=my_cloud.cloud_2d[i][1];
                dis_cout[4]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 250  &&  my_cloud.cloud_2d[i][0] <= 500){
                if(distance[5]>my_cloud.cloud_2d[i][1])
                    distance[5]=my_cloud.cloud_2d[i][1];
                dis_cout[5]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 500  &&  my_cloud.cloud_2d[i][0] <= 750){
                if(distance[6]>my_cloud.cloud_2d[i][1])
                    distance[6]=my_cloud.cloud_2d[i][1];
                dis_cout[6]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 750  &&  my_cloud.cloud_2d[i][0] <= 1000){
                if(distance[7]>my_cloud.cloud_2d[i][1])
                    distance[7]=my_cloud.cloud_2d[i][1];
                dis_cout[7]++;
            }
        }
        for(int j=0;j<8;j++){
            my_barrier.barrier[j][0]=distance[j];
            my_barrier.barrier[j][1]=dis_cout[j];
        }
        barrier_data.store(my_barrier,std::memory_order_relaxed);
    }
    return false;
}

bool Lidar3d::display_func() {
    //****
    //新建一个窗口
    pangolin::CreateWindowAndBind("compute",640,480);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);
    //定义投影和初始模型视图矩阵
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0,0,10,0,0,0,pangolin::AxisNegY));
    //在窗口中创建交互式视图
    pangolin::Handler3D handler(s_cam);

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
            .SetHandler(&handler);

    glClear(GL_COLOR_BUFFER_BIT );
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    bool display=true;
    //****
    int a=1;

    Cloud_Filtered_data my_cloud;
    Barrier_data my_barrier;

    while(a){
        std::cout<<"start read data"<<"\n";
        my_cloud=cloud_filtered_data.load(std::memory_order_relaxed);
        my_barrier=barrier_data.load(std::memory_order_relaxed);
        std::cout<<"end read data"<<"\n";

        double time_stamp = get_machine_timestamp_s() - my_cloud.time_stamp;
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout << "now_time:" << time_stamp << "\n";

        for(int i=0;i<8;i++){
            std::cout << "barrier " << i <<" :"<< my_barrier.barrier[i][0]<< " , " <<my_barrier.barrier[i][1]<< "\n";
        }

        if(display){
            double zoomout=0.002;
            //清除屏幕并激活要渲染到的视图
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            pangolin::glDrawColouredCube();//画三维方块
            // 线条长度
            pangolin::glDrawAxis(2);//三维坐标轴，红——x轴，绿——y轴，蓝——z轴

            for (int i = 0; i < my_cloud.size_t ; i++) {
                // 画map
                double red,green,blue;
                red=1-my_cloud.cloud_3d[i][0]*0.0003;
                if(red<0){
                    red=0;
                }
                green=1-my_cloud.cloud_3d[i][1]*0.001;
                if(green<0){
                    green=0;
                }
                blue=1-my_cloud.cloud_3d[i][2]*0.00003;
                if(blue<0){
                    blue=0;
                }
                glColor3f(red, green, blue);
                glPointSize(5);
                glBegin(GL_POINTS);
                glVertex3d(my_cloud.cloud_3d[i][0]*zoomout ,my_cloud.cloud_3d[i][1]*zoomout ,my_cloud.cloud_3d[i][2]*zoomout);
                // std::cout<<"i:"<<i<<",x:"<<cloud_filtered->points[i].x<<",y:"<<cloud_filtered->points[i].y<<",z:"<<cloud_filtered->points[i].z<<"\n";
            }

            //touying
            for(int i=0;i<my_cloud.size_t;i++){
                glColor3f(1, 0, 0);
                glPointSize(5);
                glBegin(GL_POINTS);
                glVertex3d(my_cloud.cloud_2d[i][0]*zoomout ,5 ,my_cloud.cloud_2d[i][1]*zoomout);
                // std::cout<<"i:"<<i<<",x:"<<cloud_filtered->points[i].x<<",y:"<<cloud_filtered->points[i].y<<",z:"<<cloud_filtered->points[i].z<<"\n";
            }
            glEnd();
            //结束
            pangolin::FinishFrame();
        }
    }
    return false;
}

bool Lidar3d::barrier_func() {
    Cloud_Filtered_data my_cloud;
    Barrier_data my_barrier;
    int a=1;
    std::chrono::microseconds period(2000*1000);
    while(a){
        auto start_time = std::chrono::system_clock::now();
        int distance[8]={9999,9999,9999,9999,9999,9999,9999,9999},cout[8]={0};
        my_cloud=cloud_filtered_data.load(std::memory_order_relaxed);
        for(int i=0;i<my_cloud.size_t;i++){
            if( my_cloud.cloud_2d[i][0] > -1000&&  my_cloud.cloud_2d[i][0] <= -750){
                if(distance[7]>my_cloud.cloud_2d[i][1])
                    distance[7]=my_cloud.cloud_2d[i][1];
                cout[7]++;
            }
            else if( my_cloud.cloud_2d[i][0] > -750 &&  my_cloud.cloud_2d[i][0] <= -500){
                if(distance[6]>my_cloud.cloud_2d[i][1])
                    distance[6]=my_cloud.cloud_2d[i][1];
                cout[6]++;
            }
            else if( my_cloud.cloud_2d[i][0] > -500 &&  my_cloud.cloud_2d[i][0] <= -250){
                if(distance[5]>my_cloud.cloud_2d[i][1])
                    distance[5]=my_cloud.cloud_2d[i][1];
                cout[5]++;
            }
            else if( my_cloud.cloud_2d[i][0] > -250 &&  my_cloud.cloud_2d[i][0] <= 0){
                if(distance[4]>my_cloud.cloud_2d[i][1])
                    distance[4]=my_cloud.cloud_2d[i][1];
                cout[4]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 0    &&  my_cloud.cloud_2d[i][0] <= 250){
                if(distance[3]>my_cloud.cloud_2d[i][1])
                    distance[3]=my_cloud.cloud_2d[i][1];
                cout[3]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 250  &&  my_cloud.cloud_2d[i][0] <= 500){
                if(distance[2]>my_cloud.cloud_2d[i][1])
                    distance[2]=my_cloud.cloud_2d[i][1];
                cout[2]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 500  &&  my_cloud.cloud_2d[i][0] <= 750){
                if(distance[1]>my_cloud.cloud_2d[i][1])
                    distance[1]=my_cloud.cloud_2d[i][1];
                cout[1]++;
            }
            else if( my_cloud.cloud_2d[i][0] > 750  &&  my_cloud.cloud_2d[i][0] <= 1000){
                if(distance[0]>my_cloud.cloud_2d[i][1])
                    distance[0]=my_cloud.cloud_2d[i][1];
                cout[0]++;
            }
        }
        for(int j=0;j<8;j++){
            my_barrier.barrier[j][0]=distance[j];
            my_barrier.barrier[j][1]=cout[j];
        }
        barrier_data.store(my_barrier,std::memory_order_relaxed);

        auto end_time   = std::chrono::system_clock::now();
        auto elapse_time  = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        if(period < elapse_time){
            std::this_thread::sleep_for(period-elapse_time);
        }
    }
    return false;
}


