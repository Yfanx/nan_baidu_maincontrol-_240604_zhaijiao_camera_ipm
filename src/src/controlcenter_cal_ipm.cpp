#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2022; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-FZ3B),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file controlcenter_cal.cpp
 * @author your name (you@domain.com)
 * @brief 智能车控制中心计算
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "recognition/track_recognition.cpp"
// #include <string>
// #include"motion_controller.cpp"
using namespace cv;
using namespace std;

class ControlCenterCal
{
public:

enum pathstype {
    
None = 0,
pathStraight, // 直入十字

pathLeftedge,  // 左斜入十字
pathRightedge, // 右斜入十字
 
pathLeftxie, 
pathRightxie, // 右斜入十字
 
pathLeftwan,  // 左斜入十字
pathRightwan, // 右斜入十字
 
};

    int controlCenter;            // 智能车控制中心（0~320）
    vector<POINT> centerEdge;     // 赛道中心点集
    uint16_t validRowsLeft = 0;   // 边缘有效行数（左）
    uint16_t validRowsRight = 0;  // 边缘有效行数（右）
    double sigmaCenter = 0;       // 中心点集的方差
    double jiaodu = 0;            // 中心点集的方差
    uint16_t validRowscenter = 0; // 边缘有效行数（右）
 uint16_t trackstdev=0;
   uint8_t tracklength = 0; // 行高（右）
int pathstype = pathstype::None;
    /**
     * @brief 控制中心计算
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */

    void controlCenterCal(TrackRecognition &track,Edgeipm &edgeipm)
    {     
    vector<POINT> centerEdgereal=edgeipm.centerEdgereal;
    POINT centermiao;
    int startrow = 0;
    int miaorow = 0,

        pathstype=edgeipm.pathstype;
        sigmaCenter = 0;
        controlCenter = COLSIMAGE / 2;
        centerEdge.clear();
        vector<POINT> v_center(4); // 三阶贝塞尔曲线
        style = "STRIGHT";
        Scalar color(255, 0, 255);    // 线条颜色（BGR格式）
        int thickness =1;   
        tracklength=MAX(track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());


        trackstdev=MAX(track.stdevRight,track.stdevLeft);

        int danbian = 0; // 左右单边标志 1--左 2---右  0--直

     // int*valid_left_right=track.validRowsCal();

      // printf("----valid_left_right-%d-- %d  -%d--%d\n" , valid_left_right[0],  valid_left_right[1], valid_left_right[2],  valid_left_right[3] );



        //track.validRowsCal();



        // printf("-1-concel-length--validRowsLeft=r--%d-- %d  ---%d\n" ,track.pointsEdgeLeft.size(), track.validRowsLeft, track.validRowsLeftend);

        // printf("-1-concel--length---validRowsRight= r---%d  ---- %d  --%d\n",  track.pointsEdgeRight.size(),track.validRowsRight, track.validRowsRightend);
       
       
        // printf("--0-  pathstype   =%d\n", pathstype);

    // pathstype =pathstype::pathStraight // 直入十字None;


    printf("--concelcal----  pathstype   =%d\n", pathstype);


        //    for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        //         {
        //             printf("\n\n-0-con i=%d --track.pointsEdgeLeft[i].x=%d  \t  y=%d\n",i,track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y); // 绿色点

        //         }

/////////////////////////////-----------------------------------------------

    if (pathstype == pathstype::pathStraight)
    {
        if (track.validRowsLeft > track.validRowsRight)

            startrow = track.validRowsLeft;
        else
            startrow = track.validRowsRight;
    }
    else if (pathstype == pathstype::pathLeftedge || pathstype == pathstype::pathLeftxie)
    {
        startrow = track.validRowsLeft;
    }
    else if (pathstype == pathstype::pathRightedge|| pathstype == pathstype::pathRightxie)
    {

        startrow = track.validRowsRight;
    }

////////////////////////////-----------------------------------------------


        // 左单边
        if (startrow <centerEdgereal.size()-5 &&(track.pointsEdgeLeft.size() >10|| track.pointsEdgeRight.size()>10))
        {
           centerEdgereal.assign(centerEdgereal.begin() +startrow, centerEdgereal.end());
           
        }
   
//////////

    //centerEdge =centerEdgereal;
// for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
//             {
//                 printf("-1-con i=%d --track.pointsEdgeLeft[i].x=%d  \t  y=%d\n", i, track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y); // 绿色点
//             }
  int i,w = 2000, h = 2000;
  cv::Size size(w, h); // 新尺寸为300x200
  cv::Mat image0,image1;      //
  cv::resize(edgeipm.imageoriginal, image0, size);
 cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
  cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);
//image0=image1.clone(); 



  for (i = edgeipm.pointsEdgeLeftnew.size() - 1; i >startrow; i--)
  {
      circle(image0,
             Point(edgeipm.pointsEdgeLeftnew[i].y + 0.5 * w, h - edgeipm.pointsEdgeLeftnew[i].x),
             5, colorLine1, 10); // 60行
      circle(
          image0,
          Point(edgeipm.pointsEdgeRightnew[i].y + 0.5 * w, h - edgeipm.pointsEdgeRightnew[i].x),
          5, colorLine2, 10); // 60行

      // circle(image0, Point(edgeipm.centerEdge[i].y, edgeipm.centerEdge[i].x), 2,  cv::Scalar(0,  0, 255), 3); // 60行

      circle(image0, Point(edgeipm.centerEdgereal[i].y + 0.5 * w, h - edgeipm.centerEdgereal[i].x), 5, cv::Scalar(0, 0, 255), 10); // 60行
  }

    // printf("pointsEdgeRightnew.size()= %d \n", edgeipm.pointsEdgeRightnew.size());
           
           
    //         for (i =0; i <centerEdgereal.size(); i++)
          
    //         {
    //          printf("Right i=%d   %d  \t%d\n",i, centerEdgereal[i].x, centerEdgereal[i].y);
             
    //          //printf("Right i=%d   %d  \t%d=\t%d \t %d   %d\n",i, edgeipm.centerEdgereal[i].x, edgeipm.centerEdgereal[i].y, edgeipm.pointsEdgeLeftnew[i].y, edgeipm.pointsEdgeRightnew[i].y, edgeipm.pointsEdgeRightnew[i].y - edgeipm.pointsEdgeLeftnew[i].y);
    //         }


 //printf("\nedgeipm.centerEdgereal[0].x, miaorow= %3.2f   %d\n", centerEdgereal[0].x, (int)(centerEdgereal.size()*0.1));


/////////////////---------------------分类选择瞄点
if (centerEdgereal[0].x>400)
 {
 miaorow =(int)(centerEdgereal.size()*0.2);

 }
 else
 {

if (centerEdgereal.back().x>1800&&centerEdgereal.size()>160)
 {
      miaorow =(int)(centerEdgereal.size()*0.99);
     
 }
 else
 if (centerEdgereal.back().x>1200&&centerEdgereal.size()>60)
 {
      miaorow =(int)(centerEdgereal.size()*0.97);
     
 }
 else
 {

  miaorow =(int)(centerEdgereal.size()*0.9);
     
 }

 }

 centermiao =centerEdgereal[miaorow];
///////////////////////--------------------------------------------




//   putText(image0, "miao:cenlen" + formatDoble2String(cenlen, 0),
//           Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x + 80),
//           FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速


//   double xielv = centerline_xielv(centerEdge);
//   printf("\n\n  xielv= %3.2f \n", xielv);

//   printf("\n\nmiaorow centerEdge[miaorow].x-y= %d \t %d \t %d \n", miaorow,
//          centerEdge[miaorow].x, centerEdge[miaorow].y);
//   printf("\n\n centermiao.x y= %3.2f \t %3.2f \n", centermiao.x,
//          centermiao.y);




float cenlen=sqrt(centermiao.x * centermiao.x + centermiao.y * centermiao.y);

jiaodu = -rad2deg(atan2(2 * 300 * centermiao.y, cenlen *cenlen));

////////////////////----------------------------------------------

//printf("miaorow=%d   %d   %d\n",miaorow,edgeipm.centerEdgereal[miaorow].x,edgeipm.centerEdgereal[miaorow].y);

//  circle(image0, Point(centerEdge[miaorow].y, centerEdge[miaorow].x), 45,
//          cv::Scalar(0, 0, 255), 3); // 60行


//---------------------绘图时加的

 miaorow = miaorow +startrow ;



//----------------------绘图时加的




circle(image0,Point(edgeipm.centerEdgereal[miaorow].y+ 0.5 * w , h-edgeipm.centerEdgereal[miaorow].x), 45,
         cv::Scalar(0, 0, 255), 5); // 60行



line(image0,Point(edgeipm.centerEdgereal[miaorow].y+ 0.5 * w,h-edgeipm.centerEdgereal[miaorow].x), Point(0.5*w,h-1),  Scalar(255, 255, 0) , 8);


line(image0, Point(edgeipm.centerEdgereal[miaorow].y+ 0.5 * w,h-edgeipm.centerEdgereal[miaorow].x),Point(0.5 * w,h-edgeipm.centerEdgereal[miaorow].x),Scalar(0, 0, 255), 4);  //200

  // -------------------------------在图层上画位置线
    color= Scalar(255, 0, 255);    // 线条颜色（BGR格式）
  thickness =2;   
line(image0, Point(0.5 * w, 0),Point(0.5 * w, h-1),  Scalar(255, 255, 0) , 2);

line(image0, Point(1, 100),Point(w-1, 100), color, thickness);  //200
line(image0, Point(1, 200),Point(w-1, 200), color, thickness);  //200
line(image0, Point(1, 300),Point(w-1, 300), color, thickness);  //200

line(image0, Point(1, 0.5 *h),Point(w-1, 0.5 *h), color, thickness);  //200
line(image0, Point(1, 0.7 *h),Point(w-1, 0.7 *h), color, thickness);  //200

line(image0, Point(1, 0.8 *h),Point(w-1, 0.8 *h), color, thickness);  //200
line(image0, Point(1, 0.9 *h),Point(w-1, 0.9 *h), color , 2);

 // -------------------------------在图层上画位置线



putText(image0, "jiaodu:  " + formatDoble2String(jiaodu , 2),
          Point(10, 0.5 *h),
          FONT_HERSHEY_PLAIN, 5, Scalar(0, 0, 255), 2); // 车速



/////////////////---------------------分类选择瞄点

  ///////////////////------------------
//   string imageName = "your_image_name.jpg";
//   MouseParams params;
//   params.img = image0;
//   params.imageName = imageName;
//   namedWindow("Image-concal");
//   setMouseCallback("Image-concal", onMouse, &params);

putText(image0,
          "miao: " + formatDoble2String(centermiao.x, 0) + "  " +
              formatDoble2String(centermiao.y, 0),
          Point(0.6*w,0.4*h),
          FONT_HERSHEY_PLAIN, 5, Scalar(0, 125, 255),4); // 车速


putText(image0, "xy0-: " + formatDoble2String(centerEdgereal[0].x, 0)+"   " +formatDoble2String(centerEdgereal[0].y,0),
          Point(20, h*0.4 ),FONT_HERSHEY_PLAIN, 5, Scalar(0, 0, 255), 4); // 车速


putText(image0, "xy-end: " + formatDoble2String(centerEdgereal.back().x, 0)+"   " +formatDoble2String(centerEdgereal.back().y,0),
          Point(20, h*0.2 ),FONT_HERSHEY_PLAIN, 5, Scalar(0, 0, 255), 4); // 车速



cv::resize(image0, image0, cv::Size((int)(w *0.25), (int)(h * 0.25)));
 //
imshow("Image_controlcenter", image0);

//waitKey(10);




        if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() > 4) // 通过双边缘有效点的差来判断赛道类型
        {
            v_center[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

            v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

            v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

            v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x + track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y + track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y) / 2};

            centerEdge = Bezier(0.03, v_center);

           //
            style = "STRIGHT";
        }





//////////////////////////////////////////

        /////////////


//    for (int i = 0; i <centerEdge.size(); i++)
//                         {
//                            printf("-1-con i=%d -centerEdge[i].x=%d  \t  y=%d\n",i,centerEdge[i].x,centerEdge[i].y); // 绿色点

//                       }
        //////////////
       // jiaodu = centerline_xielv(centerEdge);
     
         //  cout << "a==" << jiaodu << endl;

        vector<int> vec;
        for (auto p : centerEdge)
        {

            vec.push_back(p.y);
        }
        controlCenter = average(vec);
        // 加权控制中心计算
        int controlNum = 1;
        if (controlCenter > COLSIMAGE)
            controlCenter = COLSIMAGE;
        else if (controlCenter < 0)
            controlCenter = 0;

        // 控制率计算
        if (centerEdge.size() > 20)
        {
            vector<POINT> centerV;
            int filt = centerEdge.size() / 5;
            for (int i = filt; i < centerEdge.size() - filt; i++) // 过滤中心点集前后1/5的诱导性
            {
                centerV.push_back(centerEdge[i]);
            }
            sigmaCenter = sigma(centerV);
        }
        else
            sigmaCenter = 1000;

        /*  */

//------------------------------------图片上一定要加，否则会重复上一张线！！！！！！！！！
edgeipm.pointsEdgeRightnew.clear();
edgeipm.pointsEdgeLeftnew.clear();
edgeipm.centerEdge.clear();
//-----------------------------------图片上一定要加，否则会重复上一张线！！！！！！！！！





    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param centerImage 需要叠加显示的图像
     */
    void drawImage(TrackRecognition track, Mat &centerImage)
    {
        // 赛道边缘绘制
        /////////////////////////
    if (track.pointsEdgeLeft.size()<4 && track.pointsEdgeRight.size()<4)
       ;
    else
 {   



        circle(centerImage, Point(track.pointsEdgeLeft[0].y, track.pointsEdgeLeft[0].x), 5, Scalar(0, 255, 0), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y, track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x), 5, Scalar(0, 255, 0), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y, track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x), 5, Scalar(0, 255, 0), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeLeft.back().y, track.pointsEdgeLeft.back().x), 5, Scalar(0, 255, 0), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeRight[0].y, track.pointsEdgeRight[0].x), 5, Scalar(0, 255, 255), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y, track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x), 5, Scalar(0, 255, 255), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y, track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x), 5, Scalar(0, 255, 255), 2); // 绿色点
        circle(centerImage, Point(track.pointsEdgeRight.back().y, track.pointsEdgeRight.back().x), 5, Scalar(0, 255, 255), 2); // 绿色点

        ////////////////////////////

        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制中心点集
        for (int i = 0; i < centerEdge.size(); i++)
        {
            circle(centerImage, Point(centerEdge[i].y, centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
        }
}
        // 绘制加权控制中心：方向
       
        Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
        rectangle(centerImage, rect, Scalar(0, 0, 255), CV_FILLED);
        // 详细控制参数显示
        int dis = 20;
        string str;
        putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型
        //      std:string str0 = std::to_string(servoPwm);
        //	str = "PWM: " + str0;
        //  	putText(centerImage, str, Point(COLSIMAGE -30,dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, :0, 255), 1);//pwm
        
//       track.pointsEdgeLeft.assign(track.pointsEdgeLeft.begin() + track.validRowsLeft, track.pointsEdgeLeft.end());
           
        //track.pointsEdgeRight.assign(track.pointsEdgeRight.begin()+ track.validRowsRight, track.pointsEdgeRight.end());


        
        str = "tracklength: " + formatDoble2String(tracklength, 1) + " | ";
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
        



        // str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        // putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
        
        
        str = "Center: " + formatDoble2String(sigmaCenter, 2);
        putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差
        Rect roi(COLSIMAGE / 2 - 20, ROWSIMAGE - 60, 60, 30);
       // centerImage(roi) = Scalar(255, 255, 255);
        putText(centerImage, to_string(COLSIMAGE / 2 - controlCenter), Point(COLSIMAGE / 2 - 1, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 2); // 中心
    
    }

    /* @brief 边缘有效行计算：左/右
     *
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */

    TrackRecognition retrack(TrackRecognition &track)
    {

        if (track.validRowsLeftend - track.validRowsLeft > 5)
            track.pointsEdgeLeft.assign(track.pointsEdgeLeft.begin() + track.validRowsLeft, track.pointsEdgeLeft.begin() + track.validRowsLeftend - 2);
        else
            track.pointsEdgeLeft.resize(2); //.clear();

        if (track.validRowsRightend - track.validRowsRight > 5)
        {
            track.pointsEdgeRight.assign(track.pointsEdgeRight.begin() + track.validRowsRight, track.pointsEdgeRight.begin() + track.validRowsRightend - 2);
        }
        else
            track.pointsEdgeRight.resize(2);

        return track;
    }

private:
    string style = ""; // 赛道类型
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */







    /**
     * @brief 赛道中心点计算：单边控制
     *
     * @param pointsEdge 赛道边缘点集
     * @param side 单边类型：左边0/右边1
     * @return vector<POINT>
     */
    vector<POINT> centerCompute(vector<POINT> pointsEdge, int side)
    {
        int step =2;                    // 间隔尺度
        int offsetWidth = COLSIMAGE /3; // 首行偏移量
        int offsetHeight = 0;            // 纵向偏移量

        vector<POINT> center; // 控制中心集合

        if (side == 0) // 左边缘
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y > 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y + offsetWidth;
                // if (py > COLSIMAGE - 1)
                // {
                //     counter = 0;
                //     //center.emplace_back(pointsEdge[i].x - offsetHeight,COLSIMAGE - 1);
                //      center.emplace_back(pointsEdge[i].x - offsetHeight,py);
                //     // counter++;
                //     // if (counter > 2)
                //     //     break;
                // }
                // // else
                {
                    //counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }
        else if (side == 1) // 右边沿
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y < COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }




            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            
           // if (pointsEdge[rowStart].y>(offsetWidth-50))
            {

   for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y - offsetWidth;
                // if (py < 1)
                // {
                //     counter++;
                //     if (counter > 2)
                //         break;
                // }
                // else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
            }
            // else
            // {

            //  center=pointsEdge;

            // }      
         
        }

        return center;
        // return Bezier(0.2,center);
    }


    vector<POINT> centerComputestraight(TrackRecognition &track)
    {
        // int step = 4;                    // 间隔尺度
        // int offsetWidth = COLSIMAGE / 2; // 首行偏移量
        // int offsetHeight = 0;            // 纵向偏移量
        vector<POINT> v_center(4); // 三阶贝塞尔曲线
        vector<POINT> center;      // 控制中心集合
        v_center[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

        v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) / 2,
                       (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

        v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) / 2,
                       (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

        v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x + track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x) / 2,
                       (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y + track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y) / 2};

        center = Bezier(0.03, v_center);

        return center;
        // return Bezier(0.2,center);
    }




};
