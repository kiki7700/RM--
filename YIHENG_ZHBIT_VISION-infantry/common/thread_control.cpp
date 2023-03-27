/****************************************************************************
 *  Copyright (C) 2019 cz.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include "thread_control.h"

//static volatile bool end_thread_flag  = false;  // 设置结束线程标志位，负责结束所有线程。
static volatile unsigned int produce_index;     // 图像生成序号，用于线程之间逻辑
static volatile unsigned int gimbal_data_index;     // gimbal data 生成序号，用于线程之间逻辑
static volatile unsigned int consumption_index; // 图像消耗序号
static volatile unsigned int save_image_index;  // 保存图像序号

#ifdef GET_STM32_THREAD
SerialPort serial_(SERIAL_PATH,SERIAL_BAUD);                 // pc与stm32之间的串口通信
#endif 
#ifdef GET_GIMBAL_THREAD
SerialPort serial_gimbal_(GIMBAL_PATH,GIMBAL_BAUD);          // pc与陀螺仪之间的串口通信
#endif

//ThreadControl类
ThreadControl::ThreadControl()  //打开线程
{
    cout << "THREAD TASK ON !!!" << endl;
}

// 短焦长焦摄像头获取图像线程
void ThreadControl::ImageProduce()
{
    cout << " ------ SHORT/LONG CAMERA PRODUCE TASK ON !!! ------ " << endl;


#if(SHORT_CAMERA_ENABLE)
    CaptureVideo short_camera(CAMERA0_PATH, 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    short_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    short_camera.setExposureTime(0, 100);                         // 手动曝光，设置曝光时间。曝光时间：快门打开到关闭的时间间隔
    short_camera.startStream();                                  // 打开视频流
    short_camera.info();                                         // 输出摄像头信息
#endif

#if(LONG_CAMERA_ENABLE)
#ifdef GALAXY
    CameraDevice galaxy;
    galaxy.init();
#else
    CaptureVideo long_camera(CAMERA1_PATH, 3);                // 选择相机驱动文件，可在终端下输入"ls /dev" 查看. 4帧缓存
    long_camera.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);   // 设置长宽格式及使用mjpg编码格式
    long_camera.setExposureTime(0, 100);                         // 手动曝光，设置曝光时间。
    long_camera.startStream();                                  // 打开视频流
#endif
#endif

    while(1)
    {
        // 等待图像进入处理瞬间，再去生成图像
        while(produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD;
#if(SHORT_CAMERA_ENABLE==1&&LONG_CAMERA_ENABLE==1)
        if(other_param.cap_mode == 0)  //其他参数，其中使用到color颜色和cap_mode摄像头类型。0是短焦摄像头，1是长焦摄像头
#endif
#if(SHORT_CAMERA_ENABLE)
            short_camera >> image_;
#endif

#if(SHORT_CAMERA_ENABLE==1&&LONG_CAMERA_ENABLE==1)
        else if(other_param.cap_mode == 1)
#endif
#if(LONG_CAMERA_ENABLE)
#ifdef GALAXY
            galaxy.getImage(image_);    // 工业相机获取图像
#else
            long_camera >> image_;      // 普通长焦相机获得图像
#endif
#endif

        ++produce_index;
        END_THREAD;
    }
    cout << " ------ SHORT/LONG CAMERA PRODUCE TASK OUT !!! ------ " << endl;

}

//图像处理线程, 图像处理线程，用于自瞄，能量机关识别
void ThreadControl::ImageProcess()
{
    cout << " ------ IMAGE PROCESS TASK ON !!! ------" << endl;
    ArmorDetector armor_detector;  //装甲板检测
#ifdef DEBUG_PLOT
    int argc;char **argv = nullptr;
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w_ = &w;
    debug_enable_flag = true;
    armor_detector.DebugPlotInit(&w);
#endif

    BuffDetector buff_detector;  //能量机关检测
#ifdef DEBUG_PLOT
    buff_detector.DebugPlotInit(&w);
#endif

    serial_transmit_data tx_data;     // 串口发送stm32数据结构
#ifdef ARMOR_TRACK_BAR
    // 装甲板调试参数
    namedWindow("ArmorParam");
    createTrackbar("armor_gray_th", "ArmorParam", &armor_detector.gray_th_, 255);  //是opencv中的API，可在显示图像的窗口中快速创建一个滑动控件，用于手动调节阈值
    createTrackbar("armor_color_th", "ArmorParam", &armor_detector.color_th_, 255);
    createTrackbar("short_offset_x","ArmorParam",&armor_detector.short_offset_x_,200);
    createTrackbar("short_offset_y","ArmorParam",&armor_detector.short_offset_y_,200);
    createTrackbar("long_offset_x","ArmorParam",&armor_detector.long_offset_x_,200);
    createTrackbar("long_offset_y","ArmorParam",&armor_detector.long_offset_y_,200);
#endif
#ifdef BUFF_TRACK_BAR
    // 能量机关调试参数
    namedWindow("BuffParam");
    createTrackbar("buff_gray_th", "BuffParam", &buff_detector.gray_th_, 255);
    createTrackbar("buff_color_th", "BuffParam", &buff_detector.color_th_, 255);
    createTrackbar("buff_offset_x_","BuffParam",&buff_detector.buff_offset_x_,200);
    createTrackbar("buff_offset_y_","BuffParam",&buff_detector.buff_offset_y_,200);
    createTrackbar("world_offset_x","BuffParam",&buff_detector.world_offset_x_,1000);
    createTrackbar("fire_max_cnt","BuffParam",&buff_detector.auto_control.fire_task.max_cnt_,200);
    createTrackbar("reset_cnt","BuffParam",&buff_detector.auto_control.reset_task.max_cnt_,200);
    createTrackbar("repeat_time","BuffParam",&buff_detector.auto_control.fire_task.repeat_time,2000);
    createTrackbar("fire","BuffParam",&buff_detector.auto_control.fire_task.fire_flag,1);
    createTrackbar("repeat fire","BuffParam",&buff_detector.auto_control.fire_task.repeat_fire_flag,1);
#endif

#ifdef DEBUG_VIDEO  //0为自瞄视频，1为能量机关视频
#if(DEBUG_VIDEO == 0)
    VideoCapture cap(ARMOR_VIDEO_PATH);
#else
    VideoCapture cap;
    int index;
    cap.open(BUFF_VIDEO_PATH);
    //    cap.set(CV_CAP_PROP_POS_FRAMES, 9500);
//    int aaa=(int)cap.get(CV_CAP_PROP_FRAME_COUNT);
//    INFO(aaa);
#endif
#endif

    Mat image;
    float angle_x = 0.0, angle_y = 0.0, distance =  0.0;
    int command = 0;
    char key = '\0';
    bool dir = false;
    while(1)
    {
#ifndef DEBUG_VIDEO
        // 等待图像生成后进行处理
        while(produce_index - consumption_index <= 0){
            END_THREAD;
        }
        // 数据初始化
        image_.copyTo(image);//        image_.copyTo(image);
#else
        cap.read(image);
        if(key == 'f'){
            dir = !dir;
        }
        if(dir)
            flip(image, image, ROTATE_180);  //翻转一个二维矩阵（垂直、水平、垂直水平翻转） （输入矩阵、输出矩阵、0:0是围绕x轴旋转，正值是围绕y轴旋转，负值是围绕两个轴一起旋转
#endif

        if(other_param.mode == 0)  //视觉模式，0为自瞄模式，1为能量机关模式
        {
            //            ***************************auto_mode***********************************
#ifndef FORCE_CHANGE_CAMERA
            other_param.cap_mode = armor_detector.chooseCamera(1300, 1600, other_param.cap_mode);
#endif
            ++consumption_index;
            //            TIME_START(t);
//            Mat image1;
//            resize(image,image1,Size(1280,720));
            command = armor_detector.ArmorDetectTask(image, other_param);  //调用方式
            //            TIME_END(t);
            armor_detector.getAngle(angle_x, angle_y);
        }
        else
        {
            //***************************buff_mode***********************************
#ifndef FORCE_CHANGE_CAMERA
            other_param.cap_mode = 1;
#endif
            ++consumption_index;
            if(last_mode==0){
                buff_detector.readXML();  //读取调整参数

            }
            command = buff_detector.BuffDetectTask(image, other_param);
            if(command)
            {
                buff_detector.getAngle(angle_x, angle_y);
                distance = buff_detector.getDistance();
            }
        }
        if(last_mode == 1 && other_param.mode == 0){
            buff_detector.writeXML();
        }
        last_mode = other_param.mode;
        limit_angle(angle_x, 90);
        static bool fast_flag = false;
#ifdef GET_STM32_THREAD
//        INFO(command);
        tx_data.get_xy_data(-angle_x*32767/90, -angle_y*32767/90, command);
        serial_.send_data(tx_data);
#endif

#ifdef WAITKEY  //显示调试使能
#ifdef IMAGESHOW
        imshow("image", image);
#endif
        key = waitKey(WAITKEY);  //显示图像xx毫秒
        if(key == 'q')
            end_thread_flag = true;
        if(key == 'c')
        {
            if(other_param.cap_mode == 0)
                other_param.cap_mode = 1;
            else
                other_param.cap_mode = 0;
        }else if(key == 's'){
            waitKey(0);
        }else if(key == 'g'){
            fast_flag = !fast_flag;
        }else if(key == 'm'){
            other_param.mode = !other_param.mode;
        }
        END_THREAD;
#endif
    }

    cout << " ------ IMAGE PROCESS TASK OUT !!! ------" << endl;

}

#ifdef GET_STM32_THREAD
void ThreadControl::GetSTM32()   //用来接收电控发来的数据
{
    cout << " ------ STM32 DATA RECEVICE TASK ON !!! ------" << endl;
    serial_receive_data rx_data;        // 串口接收stm32数据结构
//    GimbalDataProcess GimDataPro;
    float raw_gimbal_yaw, dst_gimbal_yaw;
    bool mode = 0;
    bool color = 0;
    int yaw_offset = 0;  //yaw云台横轴，pitch云台纵轴
    int pit_offset = 0;
    while(1){
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
            END_THREAD;
        cout<<"??????????????????????????????????????????"<<endl;
        serial_.read_data(&rx_data, mode, color, yaw_offset, pit_offset);
        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        other_param.mode = mode;
        other_param.color = color;
        other_param.buff_offset_x = yaw_offset;
        other_param.buff_offset_y = pit_offset;
//        GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
//        float gimbal_data = dst_gimbal_yaw;
//        //        INFO(dst_gimbal_yaw);
//        other_param.gimbal_data = gimbal_data;

        if((gimbal_data_index%50)==0)   //生成序号，用于线程逻辑
        {
            printf("！！！！！！输出串口接收到的32信息！！！！！！！！");
            printf("Id: %d, Mode: %d, Color: %d, x:%d, y:%d\r\n", gimbal_data_index, mode, color, yaw_offset, pit_offset);
        }

#ifdef DEBUG_PLOT
        if(debug_enable_flag == true)
        {
            //            w_->addPoint(cnt,0);
            //            w_->plot();
        }
#endif
        gimbal_data_index++;
        END_THREAD;
    }
    cout << " ------ STM32 DATA RECEVICE TASK OUT !!! ------" << endl;

}
#endif

#ifdef SAVE_VIDEO_THREAD  //图像保存线程
void ThreadControl::ImageWrite()
{
    cout << " ------ IMAGE WRITE TASK ON !!! ------" << endl;
    SaveVideo writer;
    INFO(writer.getState());
    while(writer.getState()){
        while(static_cast<int>(produce_index - save_image_index) <= 0){
            END_THREAD;
        }
        Mat img_tmp;
        image_.copyTo(img_tmp);
        if(img_tmp.rows == 360)
            copyMakeBorder(img_tmp, img_tmp, 0, 120, 0, 0, BORDER_CONSTANT, Scalar::all(0));  //设置边框。源图像、目标图像、顶、底、左、右、边框类型、填充的像素值
        writer.updateImage(img_tmp);
        save_image_index++;
    }
    cout << " ------ IMAGE WRITE TASK out !!! ------" << endl;
}
#endif


void protectDate(int& a, int &b, int &c, int& d, int& e, int& f)
{
    if(a<=0)
        a = 1;
    if(b<=0)
        b = 1;
    if(c<=0)
        c = 1;
    if(d<=0)
        d = 1;
    if(e<=0)
        e = 1;
    if(f<=0)
        f = 1;
}

void limit_angle(float &a, float max)
{
    if(a > max)
        a = max;
    else if(a < -max)
        a = -max;
}
