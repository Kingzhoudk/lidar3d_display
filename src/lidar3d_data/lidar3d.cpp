//
// Created by bill on 2020/5/2.
//

#include "lidar3d.h"

int lidar3d_plan(Lidar3d_data data,int width,int height,double Ob_distance){
    // 9600像素点，水平像素点160，垂直像素点60
    int limit_height_start,limit_height_end,left_count=0,right_count=0;
    double left_total=0,right_total=0;
    limit_height_start=(60-height)/2*160+1;
    limit_height_end=9600-limit_height_start-1;
    for(int i=limit_height_start;i<=limit_height_end;i++){
        if(data.distance[i][2]<65500){
            // 图像左边
            if( (i%160) <=80 && (160-width)/2<=(i%160) ){
                left_count++;
                left_total=left_total+data.distance[i][2];
            }
                // 图像右边
            else if( (i%160) >=80 && (160+width)/2>=(i%160) ){
                right_count++;
                right_total=right_total+data.distance[i][2];
            }
        }
    }
    data.lidar_left_distance=left_total/left_count;
    data.lidar_right_distance=right_total/right_count;
    std::cout<<"left:"<<data.lidar_left_distance<<" ,right:"<<data.lidar_right_distance<<"\n";
    return 0;
}

bool Lidar3d::lidar_thread_func() {
    uint32_t i=0;
    uint32_t a = 0;
    bool display=true;
    double zoomout=0.01;

    //
    //新建一个窗口
    pangolin::CreateWindowAndBind("zhou",640,480);
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

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.2,0.0);
    pangolin::Var<double> plan_x("menu.plan_x",0);
    pangolin::Var<double> plan_y("menu.plan_y",0);
    pangolin::Var<double> now_yaw("menu.now_yaw",0);
    pangolin::Var<double> plan_yaw("menu.plan_yaw",0);
    pangolin::Var<double> now_plan_yaw("menu.now_plan_yaw",0);
    //

    /*Init Device*/
    do
    {
        // 设置测量包类型
        HPS3D_SetMeasurePacketType(ROI_DATA_PACKET);

        connect_number = HPS3D_AutoConnectAndInitConfigDevice(handle_lidar3d);
        printf("connect_number = %d\n",connect_number);
        if(connect_number == 0){
            printf("error,connect_number = %d\n",connect_number);
            break;
        }

        // 获取点云数据
        HPS3D_SetPointCloudEn(true);
        HPS3D_GetPointCloudEn();

        for(i = 0;i<connect_number;i++)
        {
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

    while(a==1)
    {
        HPS3D_SingleMeasurement(&handle_lidar3d[0]);
        for(int i=0;i<9600;i++){
            mydata.distance[i][0]=handle_lidar3d[0].MeasureData.point_cloud_data->point_data[i].x;
            mydata.distance[i][1]=handle_lidar3d[0].MeasureData.point_cloud_data->point_data[i].y;
            mydata.distance[i][2]=handle_lidar3d[0].MeasureData.point_cloud_data->point_data[i].z;
        }
        lidar3d_plan(mydata,40,10,600);
        if(display){
            plan_x=1;
            //清除屏幕并激活要渲染到的视图
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            pangolin::glDrawColouredCube();//画三维方块
            //线条长度
            pangolin::glDrawAxis(2);//三维坐标轴，红——x轴，绿——y轴，蓝——z轴

            // 画map
            for (i = 0; i < 9600 ; i++) {
                glColor3f(0.0f, 1.0f, 0.0f);
                glPointSize(1);
                glBegin(GL_POINTS);
                glVertex3d(mydata.distance[i][0]*zoomout,mydata.distance[i][1]*zoomout,mydata.distance[i][2]*zoomout);
                glEnd();
            }
            //结束
            pangolin::FinishFrame();
        }
    }

}

bool Lidar3d::display_thread_func() {

    return false;
}
