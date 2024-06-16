

// https://blog.csdn.net/Yao0203/article/details/115683047
// Win10下配置VSCode + CMake
//Windows下配置MinGW和CMake编译Makefile
//https://blog.csdn.net/qm5132/article/details/96116621
//https://blog.csdn.net/didayuye/article/details/101572914?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-101572914-blog-96116621.235%5Ev38%5Epc_relevant_sort&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-101572914-blog-96116621.235%5Ev38%5Epc_relevant_sort&utm_relevant_index=2rm
//   4 cmake .. -G "Unix Makefiles"
// PS C:\Users\nanwe\Desktop\cpp\build> rm -r *
// PS C:\Users\nanwe\Desktop\cpp\build> cmake .. -G "MinGW Makefiles"
// 9 ls
//   11 mingw32-make.exe
//   17 .\myexe.exe

#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/common.hpp"                //公共类方法文件
#include "../include/ipm.hpp"                //瞄点逆俯视查找
/////////////////////////////

#include "controlcenter_cal.cpp"                //控制中心计算类
#include "image_preprocess.cpp"                 //图像预处理类
#include "motion_controller.cpp"                //智能车运动控制类


#include "recognition/cross_recognition.cpp"    //十字道路识别与路径规划类
#include "recognition/ring_recognition.cpp"     //环岛道路识别与路径规划类
#include "recognition/track_recognition.cpp"    //赛道识别基础类
#include "recognition/parking.cpp"              //AI检测：停车区
#include "detection/qiao.cpp"                   //桥的检测

#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
using namespace std;
using namespace cv;



enum RoadType
{
  BaseHandle = 0, // 基础赛道处理
  RingHandle,     // 环岛赛道处理
  CrossHandle,    // 十字道路处理
  FreezoneHandle, // 泛行区处理
  GarageHandle,   // 车库处理
  GranaryHandle,  // 粮仓处理
  DepotHandle,    // 修车厂处理
  BridgeHandle,   // 坡道(桥)处理
  
  FarmlandHandle, // 农田区域处理
};

bool fileExists(const string& name) {
    ifstream f(name.c_str());
    return f.good();
}
int main(int argc, char **argv)
{
    // Mat image;
    // image = imread("E:\\Mydevtest\\vscode_opencv\\123.png");
    ipm.init(Size(COLSIMAGE, ROWSIMAGE),
           Size(COLSIMAGE, ROWSIMAGE)); //Size(COLSIMAGEIPM, ROWSIMAGEIPM) IPM逆透视变换初始化
     
  ImagePreprocess imagePreprocess;                // 图像预处理类
  TrackRecognition trackRecognition;              // 赛道识别

  RingRecognition ringRecognition;                // 环岛识别
  CrossroadRecognition crossroadRecognition;      // 十字道路处理

  Parking parking;          // 停车区检测类


  //
  Edgeipm  edgeipm;
  ControlCenterCal controlCenterCal;              // 控制中心计算
  MotionController motionController;              // 运动控制

  IcarShow icarShow;

  uint16_t counterRunBegin = 1;             // 智能车启动计数器：等待摄像头图像帧稳定
  RoadType roadType = RoadType::BaseHandle; // 初始化赛道类型
  uint16_t counterOutTrackA = 0;            // 车辆冲出赛道计数器A
  uint16_t counterOutTrackB = 0;            // 车辆冲出赛道计数器B
  uint16_t circlesThis = 1;                 // 智能车当前运行的圈数
  uint16_t countercircles = 0;              // 圈数计数器

 motionController.loadParams(); // 读取配置文件
controlCenterCal.validRowscenter = motionController.params.validRowscenter;


cout<<"controlCenterCal.validRowscenter ======"<<motionController.params.validRowscenter<<endl;
cout<<"motionController.params.rowCutUp ======"<<motionController.params.rowCutUp<<endl;

motionController.params.debug=1;

trackRecognition.rowCutUp = motionController.params.rowCutUp;
trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
 // garageRecognition.disGarageEntry = motionController.params.disGarageEntry;
  //  if (motionController.params.GarageEnable) // 出入库使能
  //    roadType = RoadType::GarageHandle;      // 初始赛道元素为出库


   cout<<"-------44--------motionController.params.rowCutUp ======"<<motionController.params.rowCutUp<<endl;


imagePreprocess.imageCorrecteInit(); // 图像矫正参数初始化

cv::namedWindow("icar", WINDOW_NORMAL); // 图像名称
icarShow.init(4); // 显示窗口初始化
  cout << "init done" << endl;
  
  
   cout<<"-------55---------motionController.params.rowCutUp ======"<<motionController.params.rowCutUp<<endl;

// Point2d leftIpm = ipm.homography(Point2d(3, 6));  // 透视变换
// Point2d rightIpm = ipm.homography(Point2d(6, 8)); // 透视变换

// USB摄像头初始化

// USB摄像头初始化
 // if (motionController.params.debug)

#define camera_WIN32n

#define pic_WIN32_buttonn


#ifdef camera_WIN32


    VideoCapture capture(1); // 打开默认摄像头，如果有多个摄像头，可以尝试不同的索引值
     if (!capture.isOpened()) {
       printf("can not open video device!!!\n");
       waitKey(10000); 
       return 0;
      }
capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率


#endif



  



//////////////////////////////////////////-------------------------




float delta=0;
int counter =1; // 用于记录整数值
//string dizhi0="C:/Users/nanwe/Desktop/24xiaosai/gg/train/";


string dizhi0="C:/Users/22363/Desktop/test/";

 Mat frame,frame0,invImg;


    ofstream fout("points.txt");
if (!fout.is_open())
{
      printf("can not open txt!!!\n");
       waitKey(10000);  
}

while (1)
{

//  Mat frame,frame0 ;

bool imshowRec = false; // 特殊赛道图像显示标志

#ifdef camera_WIN32

 capture>>frame;
       // 检查图像是否为空
        if (frame.empty()) {
            printf("ERROR! blank frame grabbed\n");
            break;
        }
#endif

      ////////////////-------------------选择图片输入方式
#ifdef camera_WIN32n

#ifdef pic_WIN32_button


    
      ////////////////-------------------按键图片输入方式   
     cout<<"-------77----------motionController.params.rowCutUp ======"<<motionController.params.rowCutUp<<endl;
       int key = waitKey(0); // 等待按键事件

//
printf("Key pressed:"); // 打印按键的键码值

        if (key ==49) { // 如果按下的是数字1
            counter++; // 整数值加一
        } else if (key ==50) { // 如果按下的是数字2
            counter--; // 整数值减一
        } else if (key == 27) { // 如果按下的是 ESC 键
            break; // 退出循环
        }

 #else

  printf("\n\n\n   -----input  number-----\n");    
  cin >> counter;
  
#endif  

  

namedWindow("Image Viewer", WINDOW_AUTOSIZE); // 创建一个窗口

   string filename = dizhi0+to_string(counter) + ".jpg"; // 构造文件名，这里假设图片格式为jpg




  if (fileExists(filename)) {
            frame = imread(filename); // 尝试加载图片
            frame0 =frame.clone();
            if (!frame.empty()) {

              string text = "Counter: " + to_string(counter); // 创建要显示的文本
              putText(frame0, text, Point(50, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2); // 将文本绘制到图像上
                imshow("Image Viewer",frame0); // 如果图片加载成功，则显示图片
            } else {
                Mat errorImage = Mat::zeros(200, 400, CV_8UC3); // 创建一个黑色背景的图像
                putText(errorImage, "Image load failed!", Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 显示错误信息
                imshow("Image Viewer", errorImage); // 显示错误信息图像
            }
        } else {
            Mat errorImage = Mat::zeros(200, 400, CV_8UC3); // 创建一个黑色背景的图像
            putText(errorImage, "File not found!", Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 显示错误信息
            imshow("Image Viewer", errorImage); // 显示错误信息图像
        }

   #endif   
////////////////---------------------按键图片输入方式


    // 处理帧时长监测
// 到规划末端，总计算时间

// auto start = std::chrono::high_resolution_clock::now();


    // 处理帧时长监测
// 到规划末端，总计算时间

// auto start = std::chrono::high_resolution_clock::now();

resize(frame,frame,Size(COLSIMAGE, ROWSIMAGE));


// usleep(10000);

     if (frame.empty())
    {
        cout << "Cannot open image!" << endl;

    }

  
    //--------------[02] 图像预处理
    Mat imgaeCorrect = imagePreprocess.imageCorrection(frame);         // RGB
    Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

    // 灰度化
  icarShow.setNewWindow(0, imageBinary, "imgframe"); // 添加需要显示的图像

  trackRecognition.trackRecognition(imageBinary); // 赛道线识别


 //
 printf("\n\n --trackRecognition.garageEnable.x-y =%d --\t %d\n",trackRecognition.garageEnable.x,trackRecognition.garageEnable.y);

// trackRecognition.pointsEdgeLeft.resize(trackRecognition.validRowsLeft);
// trackRecognition. pointsEdgeRight.resize(trackRecognition.validRowsRight);

// printf("----validRowsLeft =%d \n",trackRecognition.validRowsLeft );
// printf("-----validRowsRight =%d \n",trackRecognition.validRowsRight );
 Mat imageTrack;

   if (motionController.params.debug)
    {
      imageTrack=imgaeCorrect.clone();  // RGB
      trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果
      icarShow.setNewWindow(1, imageTrack, "imgTrack"); // 添加需要显示的图像
      // savePicture(imageTrack);
      //imshow("imagegg",imageTrack);
    }

    ////////////
    
      /* bool bridgeFound = FindBridge(trackRecognition);
    int counterSessionq = 0;
    if (bridgeFound) {


        printf("\n\n\n\n  this is brage \n\n\n");
        roadType = RoadType::BridgeHandle;
        counterSessionq++;
        if (counterSessionq > 10) // 上桥40场图像后失效
            //{
            //	counterRec = 0;
            //	counterSession = 0;
            //	//bridgeEnable = false;
            //	
            //}
            //else
            roadType = RoadType::BaseHandle;
    }*/
    
    ///////////////------------------------------------
    // [04] 出库和入库识别与路径规划///////////////----0717  零时注释
    //[05] 停车区检测
    if (motionController.params.GarageEnable&&0)
    {
      if (roadType == RoadType::GarageHandle ||
          roadType == RoadType::BaseHandle)
      {

        if (parking.process(trackRecognition))
        {
          roadType = RoadType::GarageHandle;
          if (parking.countExit > 20)
          {

            crossroadRecognition.reset();

            ringRecognition.reset();
            circlesThis++;
          }
          if (circlesThis >= motionController.params.circles) // 入库使能：跑完N圈

          {
            cout << "\n\n------ circlesThis==" << circlesThis << endl;

            //  uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
            sleep(1);
            printf("-----> System Exit!!! <-----\n");
            exit(0); // 程序退出
          }
        }
      }
    }



   // 左单边
//    if ( trackRecognition.stdevEdgeCal2( trackRecognition.pointsEdgeLeft,  trackRecognition.validRowsLeft, trackRecognition.pointsEdgeLeft.size()-1)>50||trackRecognition.stdevEdgeCal2( trackRecognition.pointsEdgeRight,  trackRecognition.validRowsRight, trackRecognition.pointsEdgeRight.size()-1)>50)
//      {

//  printf("\n\n-----none  straight path  stop\n"); // 绿色点
//    // if(vecmax(track.pointsEdgeRight)<320)   //if(vecmax(track.pointsEdgeRight)<320) 
//     waitKey(15000); 
// break;
//     }
///////////////////////-------------------------------4点法-逆俯视检测  240317


//auto start = std::chrono::high_resolution_clock::now();


// // //
//   ipm.homographyInv(imageTrack, invImg, cv::BORDER_CONSTANT);
//   cv::imshow("guan fangku--Transformed", invImg);
// // // // cv::waitKey(0);


///////////////////////--------------------------------4点法-逆俯视检测   240317






////////////////////////////-----------------------------------IPM获取赛道中心  --只用在环岛检测，所以放在RING里

    // edgeipm.ipm_edge(trackRecognition,imgaeCorrect);    //imageBinary
////////////////////////////-----------------------------------IPM获取赛道中心



printf("\n\n\n  -2222-controlCenterCal--validRowsLeft =%d \n",trackRecognition.validRowsLeft );


  // -----------[11] 环岛识别与处理
 if (motionController.params.RingEnable&&0) // 赛道元素是否使能
    {
      if (roadType == RoadType::RingHandle ||
          roadType == RoadType::BaseHandle)
      { 
     if (ringRecognition.ringRecognition(trackRecognition, frame,edgeipm))

     //if (ringRecognition.ringRecognition(trackRecognition, imageBinary))

        {//cout<<"\n -----ring duixing  qinrutiojian"<<endl;
          //if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
           // driver->buzzerSound(1);             // OK

          roadType = RoadType::RingHandle;
         if (motionController.params.debug)
          {
            Mat imageRing =imgaeCorrect.clone(); // 初始化图像
            ringRecognition.drawImage(trackRecognition, imageRing);
             icarShow.setNewWindow(2, imageRing, "imgRing"); // 添加需要显示的图像
                        imshowRec = true;
           // savePicture(imageRing);
          }
        else
          roadType = RoadType::BaseHandle;
          
      }
    }
    }


// 结束时间点
  //   auto endr = std::chrono::high_resolution_clock::now();

  //   // 计算耗时
  //   auto durationr = std::chrono::duration_cast<std::chrono::milliseconds>(endr - startr);

  //     // 输出结果
  //   std::cout << "\n  Time taken for task: " << durationr.count() << " milliseconds" << std::endl;
  //  std::cout << "Some output" << std::flush;
  //  printf("\n \n \n \n rinf Time taken for task: duration.count()=%d milliseconds \n\n " ,durationr.count() );

  // for (int i =0; i < trackRecognition.pointsEdgeLeft.size(); i++)
  //     {
  //      printf("left i=%d \t  %d  \t%d   \t%3.2f \n",i, trackRecognition.pointsEdgeLeft[i].x, trackRecognition.pointsEdgeLeft[i].y, trackRecognition.pointsEdgeLeft[i].slope);
  //     }


    // [12] 十字道路处理

    if (motionController.params.CrossEnable) // &&0||1赛道元素是否使能
    {
      if (roadType == RoadType::CrossHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (crossroadRecognition.crossroadRecognition(
                trackRecognition, imageBinary))
        {//cout<<"iCrossHandle ---=\n"<<i<<endl;
           //edgeipm.ipm_miaodian(trackRecognition,imgaeCorrect);             
          roadType = RoadType::CrossHandle;

          if (motionController.params.debug)
          {
            //Mat imageCross =Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
           
            Mat imageCross=imgaeCorrect.clone(); 
            crossroadRecognition.drawImage(trackRecognition, imageCross);
            icarShow.setNewWindow(2, imageCross, "imgCross"); // 添加需要显示的图像
                        imshowRec = true;
            //savePicture(imageCross);
          }
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

////////////////////////////-----------------------------------IPM获取赛道中心

//   auto startipm = std::chrono::high_resolution_clock::now();

  //edgeipm.ipm_miaodian(trackRecognition,imgaeCorrect);    //imageBinary
  //
 

  // for (int i =0; i < trackRecognition.ipmpointsEdgeRight.size(); i++)
  //     {
  //      printf("rrr i=%d \t  %d  \t%d  \n",i, trackRecognition.ipmpointsEdgeRight[i].x, trackRecognition.ipmpointsEdgeRight[i].y);
  //     }


//     auto endipm = std::chrono::high_resolution_clock::now();

   
//     auto durationipm = std::chrono::duration_cast<std::chrono::milliseconds>(endipm - startipm);

//       // 输出结果
//    std::cout << "\n  Time taken for task: " << durationipm.count() << " milliseconds" << std::endl;
//    std::cout << "Some output" << std::flush;
//    printf("\n  \n \n ----edgeipm-ipm----Time taken for task: duration.count()=%d milliseconds \n \n" ,durationipm.count() );

/////////////////////////////////--------------------------------IPM获取赛道中心
//printf("\n\n\n  -333-controlCenterCal--validRowsLeft =%d \n",trackRecognition.validRowsLeft );

  controlCenterCal.controlCenterCal(trackRecognition,edgeipm); // 根据赛道边缘信息拟合运动控制中心

//printf("-3333-controlCenterCal--validRowsLeft =%d \n",trackRecognition.validRowsLeft );


    // 智能汽车速度控制
      switch (roadType)
      {
      case RoadType::GarageHandle:
        motionController.motorSpeed =
            motionController.params.speedGarage; // 匀速控制
        break;
      case RoadType::BridgeHandle:
        motionController.motorSpeed =
            motionController.params.speedBridge; // 匀速控制
        break;
      case RoadType::RingHandle:
        motionController.motorSpeed =motionController.params.speedring; // 匀速控制

        break;
      default: // 基础巡线 | 十字 |环岛速度
            motionController.motorSpeed=motionController.params.speedLow;           
            //motionController.speedController(true, slowDown, controlCenterCal); // 变加速控制                                        
        break;
      }


//  实验舵机角度时候，注释掉,实际跑时，去掉注释
// 智能汽车方向控制
//motionController.pdController(controlCenterCal.controlCenter,controlCenterCal.jiaodu,edgeipm); // PD控制器姿态控制
 motionController.pdController(controlCenterCal); // PD控制器姿态控制
 
// 打开文件
// write_error(52, fout);
// printf("文件写入成功\n\n\n");



// write_error(motionController.servoPwm, fout);
// printf("文件写入成功\n\n\n");




// motionController.speedController(true,0.6,controlCenterCal); // 变加速控制   
printf("\n\n\n  -555-controlCenterCal--validRowsLeft =%d \n",trackRecognition.validRowsLeft );



////  智能速度控制

  //结束时间点 计算耗时
  //   auto end = std::chrono::high_resolution_clock::now();
     
  //   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  //     // 输出结果
  //  std::cout << "\n  Time taken for task: " << duration.count() << " milliseconds" << std::endl;
  //  std::cout << "Some output" << std::flush;
  //  printf("\n  \n \n -----control----Time taken for task: duration.count()=%d milliseconds" ,duration.count() );
    



    // -----------[15]调试模式下图像显示和存图
    if (motionController.params.debug)
    {
      controlCenterCal.drawImage(trackRecognition, imgaeCorrect);
      switch (roadType)
      {
      case RoadType::BaseHandle: // 基础赛道处理 // 基础赛道处理
        putText(imgaeCorrect, "[1] Track", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::RingHandle: // 环岛赛道处理 // 环岛赛道处理
        putText(imgaeCorrect, "[2] Ring", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::CrossHandle: // 十字道路处理 // 十字道路处理
        putText(imgaeCorrect, "[3] Cross", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;  
      }



        putText(imgaeCorrect, 
        "jiaodu: " + formatDoble2String(controlCenterCal.jiaodu, 2), 
        Point(COLSIMAGE - 210, 150), 
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差

 putText(imgaeCorrect, 
        "delta: " + formatDoble2String(delta, 2), 
        Point(COLSIMAGE - 210, 130), 
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2); 

      putText(imgaeCorrect,
                    "Pwm:" + formatDoble2String(motionController.servoPwm-PWMSERVOMID, 2),
                    Point(COLSIMAGE-110, 80), FONT_HERSHEY_PLAIN, 1,
                    Scalar(0, 0, 255), 2); // 车速

                  putText(imgaeCorrect,
                    "v: " + formatDoble2String(motionController.motorSpeed, 2),
                    Point(COLSIMAGE-70, 100), FONT_HERSHEY_PLAIN, 1,
                    Scalar(0, 0, 255), 2); // 车速

      string str = to_string(circlesThis) + "/" +
                   to_string(motionController.params.circles);
      putText(imgaeCorrect, str, Point(COLSIMAGE - 50, ROWSIMAGE - 20),
              cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
              CV_AA); // 显示圈数
      ////////////////// 识别范围   ////////////////////
          ////////////////// 识别范围   ////////////////////
    // 定义要画的直线参数

    Scalar color(255, 0, 255);    // 线条颜色（BGR格式）
    int thickness =1;          // 线条宽度
    
    // -------------------------------在图层上画线
 

    // 在图层上画线
  



  
    line(imgaeCorrect, Point(160,0),Point(160, 240),  Scalar(255, 255, 255) , 1);  //200
    line(imgaeCorrect, Point(1, 30),Point(319, 30), color, thickness);  //200
    line(imgaeCorrect, Point(1, 60),Point(219, 60), color, thickness);
    line(imgaeCorrect, Point(1, 120),Point(319, 120), color, thickness);
    line(imgaeCorrect, Point(1, 200),Point(319, 200), color, thickness);  //200
   
putText(imgaeCorrect,
                    formatInt2String(290, 2)+"cm" ,
                    Point(10, 232), FONT_HERSHEY_PLAIN, 1,
                    Scalar(100, 0, 255), 2); // 车速
putText(imgaeCorrect,
                     formatInt2String(330, 2)+"cm" ,
                    Point(10,  192), FONT_HERSHEY_PLAIN, 1,
                    Scalar(100, 0, 255), 2); // 车速
putText(imgaeCorrect,
                     formatInt2String(560, 2)+"cm" ,
                    Point(10,112), FONT_HERSHEY_PLAIN, 1,
                    Scalar(100, 0, 255), 2); // 车速

putText(imgaeCorrect,
                    formatInt2String(960, 2)+"cm" ,
                    Point(10,52 ), FONT_HERSHEY_PLAIN, 1,
                    Scalar(100, 0, 255), 2); // 车速
putText(imgaeCorrect,
                     formatInt2String(1300, 3)+"cm" ,
                    Point(COLSIMAGE*0.5,30), FONT_HERSHEY_PLAIN, 1,
                    Scalar(100, 0, 255), 2); // 车速
 
putText(imgaeCorrect,
                     formatInt2String(1910, 3)+"cm" ,
                    Point(COLSIMAGE*0.5,12), FONT_HERSHEY_PLAIN, 1,
                    Scalar(100, 0, 255), 2); // 车速


      //////////////////           //////////////  
      if (!imshowRec) // 保持调试图像存储顺序和显示一致性
      {
        Mat imageNone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
        icarShow.setNewWindow(2, imageNone, "None");                     // 添加需要显示的图像  
      }
      icarShow.setNewWindow(3, imgaeCorrect, "imgCorrect"); // 添加需要显示的图像                                                        //
      // if (roadType == RoadType::CrossHandle)
      // savePicture(imgaeCorrect);
      // savePicture(frame);
    // 
     icarShow.display(); // 图像窗口显示   
//enlageshow(imgaeCorrect);                //大显示屏监控用 
    // char c = waitKey(1000);
      //
       waitKey(20);
    } 
  }
 // 关闭文件
fout.close();   
}

  //////////////////
  // 要写入的文件路径
  //string file_path = "data.txt";
  // 打开文件
  //ofstream fout(file_path);
  //

  