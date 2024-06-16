
#pragma once
/**
 ********************************************************************************************************

left_ring_recgonotion


 *********************************************************************************************************
 * @file ring_recognition.cpp
 * @author Leo ()
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"
#include <thread>
#include <chrono>
using namespace cv;
using namespace std;

class RingRecognition
{
public:
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测

    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void)
    {
        //RingType ringType = RingType::RingLeft; // 环岛类型
        RingType ringType = RingType::RingNone;
        RingStep ringStep = RingStep::None;     // 环岛处理阶段
        int rowRepairLine = 0;                  // 用于环补线的点（行号）
        int colRepairLine = 0;                  // 用于环补线的点（列号）
        counterSpurroad = 0;                    // 岔路计数器
        counterShield = 0;
    }
    /**
     * @brief 环岛识别与行径规划
     *
     * @param track 基础赛道识别结果
     * @param imagePath 赛道路径图像
     */
    bool ringRecognition(TrackRecognition &track, Mat &imagePath,Edgeipm&edgeipm)
    {

        // cout<<"--------------------------left in put--------------------------------------\n"<<endl;
        // cout<<"left in put===counterShield===\n"<<counterShield<<endl;
        //  if (counterShield < 40)
        //  {
        //      counterShield++;
        //      return false;
        //  }

        // printf("\t 000--ring--track.validRowsRight=%d \n\n",track.validRowsRight);
        bool ringEnable = false;                                 // 判环标志
                                                                 // RingType ringTypeTemp = RingType::RingNone;              // 环岛类型：临时变量
        int rowBreakpointLeft = 0;                               // 边缘拐点起始行（左）
        int rowBreakpointRight = 0;                              // 边缘拐点起始行（右）
        int colEnterRing = 0;                                    // 入环点（图像列序号）
        int rowRepairRingside = track.widthBlock.size() - 1;     // 环一侧，补线起点（行号）
        int rowRepairStraightside = track.widthBlock.size() - 1; // 直道侧，补线起点（行号）
        int rowYendStraightside = track.widthBlock.size() - 1;   // 直道侧，延长补线终点（行号）
        ringPoint = POINT(0, 0);
        POINT chuleftshang = POINT{0, 0};
        bx1 = POINT{0, 0};

        int rowEnterRing = 0; // 入环点（图像列序号）
        uint16_t logo = 0, i = 0, j = 0, s1 = 0, s3 = 0, s8 = 0, s4 = 0, s5 = 0, s6 = 0, s7 = 0, cnt = 0;

        // cout<<"-----track.spurroad.size="<<track.spurroad.size()<<endl;
        // sleep(2);
        // //===============================================================修改 去掉突变值判断 Y
//   static int counter0 = 0;
//  if (ringStep != RingStep::None)
//         {
// counter0++;
// if (counter0++>100)
// {counter0 = 0;

// ringStep= RingStep::None;


// }
       // }






        // [1] 入环判断
        if (ringStep == RingStep::None && searchringroad(track, imagePath))
        {

 edgeipm.ipm_edge(track, imagePath);    //imageBinary


            printf("\n\n   --first check---ringType------=%d- ringStep--=%d  \n", ringType, ringStep);
            if (edgeipm.ipm_check_ring(imagePath))
            {
                ringStep = RingStep::Entering;
                printf("\n\n   --second-- check---ringType------=%d- ringStep--=%d  \n", ringType, ringStep);
            }
        }

        if (ringType == RingType::RingLeft)
        {
            if (ringStep == RingStep::Entering)
            {
                ///////// track.pointsEdgeRight[0].y < 320 &&
                if (track.pointsEdgeLeft.size() < 170 && track.pointsEdgeRight.size() < 170 && track.spurroad.size() < 1)
                {
                    ringStep = RingStep::Inside; // 入环补线
                    cout << "\n \n \n \n \n \n -----RingStep::Inside==" << ringStep << "\t" << endl;
                    // usleep(1000000);
                }
                else
                {
                    s1 = 0;
                    for (i = 2; i < track.pointsEdgeLeft.size(); i = i + 1)
                    {
                        if (track.pointsEdgeLeft[i].y < 3 && track.pointsEdgeLeft[i - 1].y < 3)
                            s1 += 1;
                        else
                            s1 = 0;
                        if (s1 > 10)
                        {
                            printf("-----Right ring=s4=%d \t i= %d=\t%d\t%d \n", s4, i, track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y);
                            break;
                        }
                    }

                    if (s1 > 0)
                    {
                        for (j = i; j < track.pointsEdgeLeft.size() - 40; j++)
                        {
                            printf("Right ring slope =s4=%3.2f=---%d\t%d\t%d \n", track.pointsEdgeLeft[j].slope, track.pointsEdgeLeft[j].y - track.pointsEdgeLeft[j].y, track.pointsEdgeLeft[j].x, track.pointsEdgeLeft[j].y);
                        }

                        for (j = i; j < track.pointsEdgeLeft.size() - 10; j++)
                        {                                                                                                            // track.pointsEdgeLeft[j].y>100&&track.widthBlock[j +3].y<180&&
                            if ((track.widthBlock[j].y - track.widthBlock[j + 1].y) > 30 && track.pointsEdgeLeft[j + 3].slope < -12) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
                            {
                                rowRepairLine = j + 3;
                                ringPoint = track.pointsEdgeLeft[rowRepairLine];
                                printf("\n\n+++++++++++ Right ring shang ---track.widthBlock[rowRepairLine].y=%d\n", track.widthBlock[rowRepairLine].y);
                                printf("\n\n Right ring= \t ringPoint.x= %d=\t%d\t %d \n", ringPoint.x, track.pointsEdgeLeft[rowRepairLine].x, track.pointsEdgeLeft[rowRepairLine].y);
                                printf("\n\n Right ringPoint.x  slope= %d=\t%3.2f\t %3.2f \n", ringPoint.x, track.pointsEdgeLeft[rowRepairLine].slope, track.pointsEdgeLeft[rowRepairLine + 1].slope);
                                break;
                            }
                        }
                    }

                    if (ringPoint.x > 60) // rowRepairLine > 10 &&
                    {

                        ////////// 240223  添加的，--

                        //
                        printf("\n\n ----Right ring= \t ringPoint.x= %d=\t%d\t %d \n", ringPoint.x, track.pointsEdgeLeft[rowRepairLine].x, track.pointsEdgeLeft[rowRepairLine].y);

                        if (track.spurroad.size() > 0 && ringPoint.x < track.spurroad[0].x && ringPoint.y > track.spurroad[0].y && (track.spurroad[0].x - ringPoint.x) < 80)
                            ringPoint = track.spurroad[0];

                        //   {ringPoint.x= track.spurroad[0].x;
                        //  ringPoint.y= track.spurroad[0].y;}

                        /////////  240223  添加的，--

                        // if (ringPoint.y > 220)
                        //     ringPoint.y = ringPoint.y * 0.6;

                        //////////////////
                        int rightstart = track.validRowsRight;

                        if ((track.pointsEdgeRight[rightstart].x - ringPoint.x) > 50)
                        {
                            if (rightstart < rowRepairLine * 0.2)
                                rightstart = rowRepairLine * 0.2;
                            bx1 = track.pointsEdgeRight[rightstart];
                        }
                        else if (ringPoint.x > 150)
                        {

                            rightstart = 0;
                            bx1 = track.pointsEdgeRight[rightstart];
                            bx1.y = ringPoint.y + 30;
                        }
                        else
                        {
                            rightstart = 0;
                            bx1 = track.pointsEdgeRight[rightstart];
                        }
                        track.pointsEdgeRight.resize(rowRepairLine);
                        track.pointsEdgeLeft.resize(rowRepairLine);
                        cout << "==========================================="
                             << "开始布线操作" << endl;
                        ///////////////////LEFT EDGE BUXIAN

                        float k = 1.0 * (ringPoint.y - bx1.y) / (ringPoint.x - bx1.x);
                        ////////////////////////
                        float b = ringPoint.y - k * ringPoint.x;

                        printf("\n\n Right rightstart= %d  k= %3.2f=\tb=%3.2f \n", rightstart, k, b);

                        for (int i = rightstart; i < rowRepairLine; i++)
                        {
                            track.pointsEdgeRight[i].y = k * track.pointsEdgeRight[i].x + b;

                            // printf("buxian--- Right i= %d  k= track.pointsEdgeRight[i].y=%d  \n",i,track.pointsEdgeRight[i].y);

                            // if (track.pointsEdgeRight[i].y < 3 || track.pointsEdgeRight[i].y > 318)
                            //     break;
                        }
                    }
                }
            }

            //////////////// 左出环补线/-----------------------RingStep::Inside///////////////-------------------入环出程序

            if (ringStep == RingStep::Inside)
            {

                // rowRepairLine = 0;

                // printf("buxian--- Right track.validRowsRight= %d  k= track.pointsEdgeRight[i].y=%d  \n", track.validRowsRight, track.pointsEdgeRight[0].y);

                if (track.validRowsLeft > 100 && track.validRowsRight > 70 && track.validRowsLeft - track.validRowsRight > 40)
                {
                    printf("----buxian--- Right track.validRowsRight\n");
                    ringStep = RingStep::Exiting; // 入环补线
                    cout << "\n \n \n \n \n \n -----RingStep::Exiting==" << ringStep << "\t" << endl;
                }

                printf("--22222--RingStep::Inside\n");

                if (ringStep != RingStep::Exiting)
                {

                    for (int i = 2; i < (track.pointsEdgeRight.size() - 10); ++i)
                    {
                        if (track.pointsEdgeRight.size() > 120 && track.pointsEdgeRight[i - 1].slope > 0 && track.pointsEdgeRight[i].slope > 0 && track.pointsEdgeRight[i + 1].slope > 0 && track.pointsEdgeRight[i + 4].slope < 0 && track.pointsEdgeRight[i + 5].slope < 0)
                        {

                            rowRepairLine = i + 1;
                            ringPoint = track.pointsEdgeRight[rowRepairLine];
                            cout << "----右边出环岛点坐标==" << i << endl;
                            printf("\n\n Right exit--- ring=rowRepairLine=%d \t ringPoint.x= %d=\t%d\t%d \n", rowRepairLine, ringPoint.x, ringPoint.y, track.pointsEdgeLeft[j].x);

                            break;
                        }
                    }

                    if (rowRepairLine > 5 && ringPoint.x > 60)
                    {

                        track.pointsEdgeRight.resize(rowRepairLine);
                        if (track.validRowsLeft > 10)
                        {
                            chuleftshang = track.pointsEdgeLeft[track.validRowsLeft];
                            chuleftshang.x += 30;
                            track.pointsEdgeLeft.resize(track.validRowsLeft);
                        }
                        else
                        {
                            track.pointsEdgeLeft.resize(3);
                            chuleftshang = ringPoint;
                            chuleftshang.x -= 20;
                            chuleftshang.y = 1;
                        }
                        POINT temppoint;
                        float k = 1.0 * (ringPoint.y - chuleftshang.y) / (1.0 * ringPoint.x - chuleftshang.x);
                        float b = ringPoint.y - k * ringPoint.x;
                        if (chuleftshang.x < ringPoint.x)
                        {
                            for (int i = ringPoint.x; i > chuleftshang.x; --i)
                            {
                                temppoint.x = i;
                                temppoint.y = k * i + b;
                                track.pointsEdgeRight.push_back(temppoint);
                            }
                        }
                    }
                    else if (track.validRowsRight > 90 && track.validRowsLeft > 70 && track.validRowsLeft - track.validRowsRight < 41)
                    {
                        chuleftshang = track.pointsEdgeLeft[track.validRowsLeft - 2];
                        ringPoint = track.pointsEdgeRight[6];
                        track.pointsEdgeRight.resize(5);
                        POINT temppoint;
                        float k = 1.0 * (ringPoint.y - chuleftshang.y) / (1.0 * ringPoint.x - chuleftshang.x);
                        float b = ringPoint.y - k * ringPoint.x;
                        if (chuleftshang.x < ringPoint.x)
                        {
                            for (int i = ringPoint.x; i > chuleftshang.x; --i)
                            {
                                temppoint.x = i;
                                temppoint.y = k * i + b;
                                track.pointsEdgeRight.push_back(temppoint);
                            }
                        }
                        cout << "\n \n \n \n \n \n -----RingStep::Exiting==" << ringStep << "\t" << endl;
                    }
                }
            }

            ///////////

            if (ringStep == RingStep::Exiting)
            {

                static int counter = 0; // 静态局部变量
                counter++;              // 每次调用函数时增加1
                std::cout << "Counter is " << counter << std::endl;
                printf("\n\n Right exit--- Counter is\t%d \n", counter);
                ///-------------帧率累计计数   与速度强相关
                // 当计数器达到3时，执行一些特定的行为
                //         if (counter>35)

                //         {
                // ringStep =RingStep::Finish ; // 入环补线
                // counter=0;
                // printf("\n\n RingStep::Finish  Right exit--- Counter is\t%d \n", counter );
                //         }

                ///-------------帧率累计计数

                if (track.validRowsLeft < 10 && track.validRowsLeft < track.pointsEdgeLeft.size() * 0.3 && track.widthBlock[track.validRowsLeft + 1].y > track.widthBlock[track.validRowsLeft + 3].y && track.pointsEdgeLeft[track.validRowsLeft].x > 230)
                {
                    ringStep = RingStep::Finish; // 入环补线
                }
                //
                else

                    track.pointsEdgeLeft.resize(2);
            }

            //   printf("--2--);
            // 出环，切回正常循迹
        }

        ////////////////////  ---rightring_rec

        if (ringType == RingType::RingRight)
        {
           if (ringStep == RingStep::Entering)
             {
                //  for (int i =track.validRowsRight; i < track.pointsEdgeRight.size() - 1; i++)
                // {
                //    if (track.pointsEdgeRight[i].y<319)
                //       printf("Right ring=%d=\t= %d=\t%d \t %3.2f \n", i, track.pointsEdgeRight[i].x, track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].slope);
                //     else
                //      break;
                // }

   
                           

                if (track.pointsEdgeLeft.size() < 170 && track.pointsEdgeRight.size() < 170 && track.spurroad.size() < 1)
                {
                    ringStep = RingStep::Inside; // 入环补线
                    cout << "\n \n \n \n \n \n -----RingStep::Inside==" << ringStep << "\t" << endl;
                    // usleep(1000000);
                }
                else
                {
                    s1 = 0;
                    for (i = 2; i < track.pointsEdgeRight.size()-1; i = i + 1)
                    {
                        if (track.pointsEdgeRight[i].y > COLSIMAGE - 3 && track.pointsEdgeRight[i - 1].y > COLSIMAGE - 3)

                            s1 += 1;
                        else
                            s1 = 0;
                        if (s1 > 10)
                        {
                            printf("---Right ring=s4=%d \t i= %d=\t%d\t%d \n", s4, i, track.pointsEdgeRight[i].x, track.pointsEdgeRight[i].y);
                            break;
                        }
                    }

                    if (s1 > 0)
                    {
                        // for (j =track.pointsEdgeLeft.size() -4; j>i-10 ; j--)
                        //   {
                        //   printf("Right ring slope =s4=%3.2f=---%d\t%d\t%d \n",track.pointsEdgeRight[j].slope,track.pointsEdgeRight[j].y-track.pointsEdgeRight[j].y, track.pointsEdgeRight[j].x, track.pointsEdgeRight[j].y);
                        //   }

                        for (j = i; j < track.pointsEdgeRight.size() - 10; j++)
                        {                                                                                                           // track.pointsEdgeLeft[j].y>100&&track.widthBlock[j +3].y<180&&
                            if ((track.widthBlock[j].y - track.widthBlock[j + 1].y) > 30 &&track.pointsEdgeRight[j+5].slope>0&&track.pointsEdgeRight[j + 5].slope<1) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
                            {
                                rowRepairLine = j + 3;
                                ringPoint = track.pointsEdgeRight[rowRepairLine];
                                //
                                printf("\n\n+++++++++++ Right ring shang ---track.widthBlock[rowRepairLine].y=%d\n", track.widthBlock[rowRepairLine].y);
                                printf("\n\n Right ring= \t ringPoint.x= %d=\t%d\t %d \n",rowRepairLine, track.pointsEdgeRight[rowRepairLine].x, track.pointsEdgeRight[rowRepairLine].y);

                                printf("\n\n Right ringPoint.x  slope= %d=\t%3.2f\t %3.2f \t %3.2f\n", ringPoint.x, track.pointsEdgeRight[rowRepairLine].slope, track.pointsEdgeRight[rowRepairLine + 1].slope,track.pointsEdgeRight[rowRepairLine + 2].slope);

                                break;
                            }
                        }
                    }
                
                     //  printf("\n-000---Right ring track.pointsEdgeLeft.size()=--%d\t track.spurroad.size()=%d\\n", track.pointsEdgeRight.size(), track.spurroad.size());

                    // 当前时间加上1秒
                
                       //  std::this_thread::sleep_for(std::chrono::seconds(100));
// printf("\n\n -ringPoint.x---track.spurroad[0].xy-=\t%d=\t%d\t %d \n",ringPoint.x, track.spurroad[0].x, track.spurroad[0].y);
                      
                    if (ringPoint.x >50 || ringPoint.x <=50 && track.spurroad.size()>0&&track.spurroad[0].x > 90 && track.spurroad[0].y < 160) // rowRepairLine > 10 &&
                    {
                        // if (ringPoint.y > 220)
                        //     ringPoint.y = ringPoint.y * 0.6;
                        ////////////////////////////////////////////////////////////////////////------------------------------------------

                        // 
                        
                        // printf("\n\n ----Right ring= \t ringPoint.x= %d=\t%d\t %d \n", ringPoint.x, track.pointsEdgeLeft[rowRepairLine].x, track.pointsEdgeLeft[rowRepairLine].y);

                        if (track.spurroad.size() > 0 && ringPoint.x < track.spurroad[0].x && ringPoint.y <= track.spurroad[0].y && (track.spurroad[0].x - ringPoint.x) < 100 ||track.spurroad.size() > 0 && track.spurroad[0].x > 190 && track.spurroad[0].y < 160)
                            ringPoint = track.spurroad[0];
 if(ringPoint.x==0&&track.spurroad.size() > 0)
 ringPoint = track.spurroad[0];
                        ///////
                        int rightstart = track.validRowsLeft;
                        if (ringPoint.x > 150)
                        {

                            rightstart = 0;
                            bx1 = track.pointsEdgeLeft[rightstart];

                            if (ringPoint.y > 30)
                                bx1.y = ringPoint.y - 30;
                            else
                                bx1.y = ringPoint.y;
                        }
                        else
                        {
                            rightstart = 0;
                            bx1 = track.pointsEdgeLeft[rightstart];
                        } // if (ringPoint.x > 100)

                        track.pointsEdgeRight.resize(rowRepairLine);
                        track.pointsEdgeLeft.resize(rowRepairLine);
                        ///////////
                        cout << "==========================================="
                             << "开始布线操作" << endl;
                        /////////////////////////////////////////////////////////////////---------------------------------------------------

                        //////////////////////// ///////////////////LEFT EDGE BUXIAN

                        float k = 1.0 * (ringPoint.y - bx1.y) / (ringPoint.x - bx1.x);
                        float b = ringPoint.y - k * ringPoint.x;
                        printf("\n\n Right rightstart= %d  k= %3.2f=\tb=%3.2f \n", rightstart, k, b);
                        for (int i = rightstart; i < rowRepairLine; i++)
                        {
                            track.pointsEdgeLeft[i].y = k * track.pointsEdgeLeft[i].x + b;

                            // printf("buxian--- Right i= %d  k= track.pointsEdgeRight[i].y=%d  \n",i,track.pointsEdgeRight[i].y);

                            // if (track.pointsEdgeRight[i].y < 3 || track.pointsEdgeRight[i].y > 318)
                            //     break;
                        }
                    }
                }
            }

            //////////////// ----右出环补线/-----------------------RingStep::Inside///////////////-------------------入环出程序

            if (ringStep == RingStep::Inside)
            {
                rowRepairLine = 0;
                printf("buxian--- Right track.validRowsRight= %d  k= track.pointsEdgeRight[i].y=%d  \n", track.validRowsRight, track.pointsEdgeRight[i].y);

                ////////////
                // if (track.validRowsLeft > 100 && track.validRowsRight > 70 && track.validRowsLeft - track.validRowsRight > 40)
                //             {

                ////////////

                if (track.validRowsLeft > 70 && (track.pointsEdgeLeft[track.validRowsLeft].x - track.pointsEdgeLeft.back().x > 80 || track.validRowsRight > 100 && track.validRowsRight - track.validRowsLeft > 40))
                {

                    ringStep = RingStep::Exiting; // 入环补线
                    cout << "\n \n \n \n \n \n -----RingStep::Exiting==" << ringStep << "\t" << endl;
                }

                if (ringStep != RingStep::Exiting)
                {

                    for (int i = 0; i < track.pointsEdgeLeft.size() - 1; i++)
                    {

                        printf("LEFT ring=%d=\t= %d=\t%d\t \n", i, track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y);
                    }

                    for (int i = 1; i < (track.pointsEdgeLeft.size() - 15); ++i)
                    {
                        // if (track.pointsEdgeLeft.size() > 120 && track.pointsEdgeLeft[i - 1].slope < 0 && track.pointsEdgeLeft[i].slope < 0 && track.pointsEdgeLeft[i + 1].slope < 0 && track.pointsEdgeLeft[i + 4].slope > 0 && track.pointsEdgeLeft[i + 5].slope > 0)
                        // {
 if (track.pointsEdgeLeft.size() > 120 &&i<track.pointsEdgeLeft.size()-60&& track.pointsEdgeLeft[i].y<track.pointsEdgeLeft[i+5].y && track.pointsEdgeLeft[i+2].y<track.pointsEdgeLeft[i+6].y&& track.pointsEdgeLeft[i+5].y>track.pointsEdgeLeft[i+9].y  &&track.pointsEdgeLeft[i+7].y>track.pointsEdgeLeft[i+12].y)
                        {
                            rowRepairLine = i +5;
                            ringPoint = track.pointsEdgeLeft[rowRepairLine];
                            cout << "----右边出环岛点坐标==" << i << endl;
                            printf("\n\n Left exit--- ring=rowRepairLine=%d \t ringPoint.x= %d=\t%d\t%d \n", rowRepairLine, ringPoint.x, ringPoint.y, track.pointsEdgeLeft[j].x);

                            break;
                        }
                    }

                    if (rowRepairLine > 1 && ringPoint.x > 60)
                    {

                        track.pointsEdgeLeft.resize(rowRepairLine);
                        if (track.validRowsRight > 10)
                        {

                            chuleftshang = track.pointsEdgeRight[track.validRowsRight];
                            chuleftshang.x += 30;
                            track.pointsEdgeRight.resize(track.validRowsRight);
                        }
                        else
                        {

                            track.pointsEdgeRight.resize(3);

                            chuleftshang = ringPoint;
                            chuleftshang.x -= 20;
                            chuleftshang.y = 1;
                        }
                        POINT temppoint;
                        float k = 1.0 * (ringPoint.y - chuleftshang.y) / (1.0 * ringPoint.x - chuleftshang.x);
                        float b = ringPoint.y - k * ringPoint.x;
                        if (chuleftshang.x < ringPoint.x)
                        {
                            for (int i = ringPoint.x; i > chuleftshang.x; --i)
                            {
                                temppoint.x = i;
                                temppoint.y = k * i + b;
                                track.pointsEdgeLeft.push_back(temppoint);
                            }
                        }
                    }
                    else if (track.validRowsRight > 70 && track.validRowsLeft > 90 && track.validRowsRight - track.validRowsLeft < 41 || averagepoint(track.pointsEdgeLeft) < 20 && track.validRowsRight > 70)

                    {
                        chuleftshang = track.pointsEdgeRight[track.validRowsRight - 2];

                        ringPoint = track.pointsEdgeLeft[6];

                        track.pointsEdgeLeft.resize(5);
                        POINT temppoint;
                        float k = 1.0 * (ringPoint.y - chuleftshang.y) / (1.0 * ringPoint.x - chuleftshang.x);
                        float b = ringPoint.y - k * ringPoint.x;
                        if (chuleftshang.x < ringPoint.x)
                        {
                            for (int i = ringPoint.x; i > chuleftshang.x; --i)
                            {
                                temppoint.x = i;
                                temppoint.y = k * i + b;
                                track.pointsEdgeLeft.push_back(temppoint);
                            }
                        }
                        cout << "\n \n \n \n \n \n -----RingStep::Exiting==" << ringStep << "\t" << endl;
                    }
                }
            }

            ///////////

            if (ringStep == RingStep::Exiting)
            {

                static int counter = 0; // 静态局部变量
                counter++;              // 每次调用函数时增加1
                std::cout << "Counter is " << counter << std::endl;
                printf("\n\n Right exit--- Counter is\t%d \n", counter);

                ///-------------帧率累计计数   与速度强相关
                // 当计数器达到3时，执行一些特定的行为
                        if (counter>10)
                        {

                ringStep =RingStep::Finish ; // 入环补线
                counter=0;
                printf("\n\n RingStep::Finish  Right exit--- Counter is\t%d \n", counter );
                        }
                        else {
                          if (track.validRowsRight < 10 && track.validRowsRight < track.pointsEdgeRight.size() * 0.3 && track.widthBlock[track.validRowsRight + 1].y > track.widthBlock[track.validRowsRight + 3].y && track.pointsEdgeRight[track.validRowsRight].x > 230)
                {
                    ringStep = RingStep::Finish; // 入环补线
                }
                //
                else

                    track.pointsEdgeRight.resize(2);
                        }

                ///-------------帧率累计计数

              
            }
        }
        ///////////////////////////////

        if (ringStep == RingStep::Finish)
        {
            {
                ringStep = RingStep::None;
            }
        }

        // printf("-44444--RingStep::Inside\n");

        // //--------------------------------临时测试----------------------------------
        // 返回识别结果
        if (ringStep == RingStep::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 绘制环岛识别图像
     *
     * @param ringImage 需要叠加显示的图像
     */
    void drawImage(TrackRecognition track, Mat &ringImage)
    {

        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(ringImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(ringImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(ringImage, Point(track.spurroad[i].y, track.spurroad[i].x), 5,
                   Scalar(0, 0, 255), -1); // 红色点
        }

        putText(ringImage, to_string(_ringStep) + " " + to_string(_ringEnable) + " " + to_string(_tmp_ttttt),
                Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

        putText(ringImage, to_string(_index), Point(80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

        putText(ringImage, to_string(track.validRowsRight) + " | " + to_string(track.stdevRight),
                Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);
        putText(ringImage, to_string(track.validRowsLeft) + " | " + to_string(track.stdevLeft),
                Point(30, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);

        string ringtypeh;
        if (ringType == RingType::RingLeft)
            ringtypeh = "RingLeft" + formatInt2String(ringStep, 2);
        else if (ringType == RingType::RingRight)
            ringtypeh = "RingRight" + formatInt2String(ringStep, 2);

        putText(ringImage, ringtypeh, Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型

        circle(ringImage, Point(ringPoint.y, ringPoint.x), 15, Scalar(0, 0, 255), 2); // 红色点
        if (track.spurroad.size() > 0)
            circle(ringImage, Point(track.spurroad[0].y, track.spurroad[0].x), 25, Scalar(255, 0, 0), 2); // 红色点

        circle(ringImage, Point(bx1.y, bx1.x), 25, Scalar(0, 255, 0), 2); // 红色点

        // void cv::circle (InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE_8, int shift=0)
    }

private:
    uint16_t counterSpurroad = 0; // 岔路计数器
    // 临时测试用参数
    int _ringStep;
    int _ringEnable;
    int _tmp_ttttt;
    int _index = 0;
    POINT ringPoint = POINT(0, 0);
    POINT bx1 = POINT{0, 0};
    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Exiting,  // 出环
        Finish    // 环任务结束162
    };

    RingType ringType = RingType::RingLeft; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）

    bool searchringroad(TrackRecognition &track, Mat &imagePath)
    {

        vector<POINT> vecr(track.pointsEdgeRight.begin() + track.validRowsRight, track.pointsEdgeRight.end());
        vector<POINT> vecl(track.pointsEdgeLeft.begin() + track.validRowsLeft, track.pointsEdgeLeft.end());
        //(track.stdevEdgeCal(vecl, ROWSIMAGE)>50&&track.stdevEdgeCal(vecr, ROWSIMAGE)>50)&&||vecl.size() < ROWSIMAGE * 0.8 || vecl.size() < ROWSIMAGE * 0.8
        if (track.validRowsRight > ROWSIMAGE * 0.8 || track.validRowsLeft > ROWSIMAGE * 0.8 || track.pointsEdgeLeft.size() < ROWSIMAGE * 0.8 || track.pointsEdgeRight.size() < ROWSIMAGE * 0.8)

        {
            return false;
        }

        //    for (int i = 0; i < track.pointsEdgeRight.size() - 1; i++)
        //     {

        //         printf("Right ring=%d=\t= %d=\t%d\t \n", i, track.pointsEdgeRight[i].x, track.pointsEdgeRight[i].y);
        //     }
        printf("\n ring-track.pointsEdgeRight.size()-vecl.size() )=%d --%d \n\n", track.pointsEdgeRight.size(), vecr.size());

        if (vecl.size() > ROWSIMAGE * 0.6)
        {
            vecl.erase(vecl.end() - 30, vecl.end());
        }

        //  if (track.stdevEdgeCal(vecl, ROWSIMAGE)>50&&track.stdevEdgeCal(vecr, ROWSIMAGE)>50)
        if (vecr.size() > ROWSIMAGE * 0.6)
        {
            // vecr.assign(vecr.begin(), vecr.end() - 50);
            vecr.erase(vecr.end() - 30, vecr.end());
            printf("\n 6666666666---ring-track.pointsEdgeRight.size()-vecl.size() )=%d --%d \n\n", track.pointsEdgeRight.size(), vecr.size());
        }

        //  for (int i = 0; i < vecr.size() - 1; i++)
        //             {

        //                 printf("Right ring--vecr=%d=\t= %d=\t%d\t \n", i, vecr[i].x, vecr[i].y);
        //             }

        printf("\n--- ring-trvecl.size(), vecr.size())=%d --%d \n\n", vecl.size(), vecr.size());
        // if (vecl.size() > ROWSIMAGE * 0.8&&track.stdevEdgeCal(vecl, ROWSIMAGE) > 50 && track.pointsEdgeRight.size() > ROWSIMAGE * 0.8)
        // {

        //     printf("\n ring-track.pointsEdgeLeft.size()-sigma( -Left)=%d --%3.2f \n\n", track.pointsEdgeLeft.size(), track.stdevEdgeCal(vecl, ROWSIMAGE));

        //     printf("\n ring-track.pointsEdgeRight.size()-sigma( -right)=%d --%3.2f \n\n", track.pointsEdgeRight.size(), track.stdevEdgeCal(vecr, ROWSIMAGE));

        //
        printf("\n ----ring--track.stdevEdgeCal(vecl, ROWSIMAGE) -right)=%3.2f= --%3.2f \n\n", track.stdevEdgeCal(vecl, ROWSIMAGE), track.stdevEdgeCal(vecr, ROWSIMAGE));
        //     // sleep(1);
        // }
        // track.stdevLeft<100||track.stdevRight<100||

        uint16_t logo = 0, i = 0, j = 0, s1 = 0, s3 = 0, s8 = 0, s4 = 0, s5 = 0, s6 = 0, s7 = 0, cnt = 0;

        /////////////////////  -----  右单边

        // 边缘有效行优化
        // if (track.pointsEdgeLeft.size()>150)    // cout<<"s1=0000000000"<<endl;

        /// left  ring
        // if (track.pointsEdgeLeft[i].x > 180)
        if (vecr.size() > ROWSIMAGE * 0.6 && track.stdevEdgeCal(vecl, ROWSIMAGE) > 100 && track.stdevEdgeCal(vecr, ROWSIMAGE) < 30)
        {
            for (j = 0; j < vecr.size(); j++)
            {
                // cout<<"s1=11111111111"<<endl;

                if (vecr[j].y - vecr[j + 10].y > 3)
                {
                    s1 += 1;
                }
                else
                    s1 = 0;
                if (s1 > 140)
                {
                    logo = 1; // leftring tiaojian1
                    // ringType = RingType::RingLeft;
                    //
                    printf("\n\n\n  -KKK---------------RingType::RingLeft \n");
                    break;
                }
            }
        }

        ///--------------------------------------------------------------right  ring

        // if (track.pointsEdgeLeft[i].x > 180)

        //  for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() -40; j++)
        //         {
        //            printf(" track.pointsEdgeLeft[j].y-----------%d---%d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);

        //                 //printf("\n\n\n  -KKK---------------RingType::RingLeft \n");

        //         }


        if (logo < 1)
        {

printf("\n\n\n-xxxx--RingRight=track.stdevEdgeCal(vecl, ROWSIMAGE) %3.2f =\t%3.2f  \n\n\n", track.stdevEdgeCal2(track.pointsEdgeLeft, track.validRowsLeft,track.pointsEdgeLeft.size()-1), track.stdevEdgeCal2(track.pointsEdgeRight, track.validRowsRight,track.pointsEdgeRight.size()-1));

            /// left  ring
            s1 = 0;
            if (vecl.size() > ROWSIMAGE * 0.6 &&track.stdevEdgeCal2(track.pointsEdgeLeft, track.validRowsLeft,track.pointsEdgeLeft.size()-1) <50 &&track.stdevEdgeCal2(track.pointsEdgeRight, track.validRowsRight,track.pointsEdgeRight.size()-1) >200)
            {
                for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 20; j++)
                {
                    // cout<<"s1=11111111111"<<endl;
                    if (track.pointsEdgeLeft[j + 20].y - track.pointsEdgeLeft[j].y > 5)
                    {
                        s1 += 1;
                    }
                    else
                        s1 = 0;
                    if (s1 > 1)
                    {
                        logo = 2; // ringtring  tiaojian
                                  // ringType = RingType::RingRight;
                                  //
                        printf("\n\n\n --oo------- RingType::RingRight\n");
                        break;
                    }
                }
            }
        }
        ///////////////////-----  右环岛判断特征

        s4 = 0;
        s5 = 0;
        s6 = 0;
        s7 = 0;

        if (logo == 2)
        {
            // for (int i = 0; i < track.pointsEdgeRight.size() - 1; i++)
            // {

            //     printf("--Right ring=%d=\t= %d=\t%d\t %3.2f \n", i, track.pointsEdgeRight[i].x, track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].slope);
            // }

           // printf("\n\n Right ring= track.validRowsRight --\t%d\t \n", track.validRowsRight);

            if (track.validRowsRight >30)
            {
                s4 = 10;
                i = track.validRowsRight;
            }
            else
            {

                for (i = track.validRowsRight; i < track.pointsEdgeRight.size(); i++)
                {
                    if (track.pointsEdgeRight[i].y > COLSIMAGE - 3 && track.pointsEdgeRight[i + 1].y > COLSIMAGE - 3)

                        s4 += 1;
                    else
                        s4 = 0;
                    if (s4 >20)
                    {
                        printf("\n\n Right ring= s4=\t%d\t \n", s4);

                        break;
                    }
                }
            }
            printf("\n\n ----Right ring= s4=\t%d\t i=%d\t \n", s4,i);
            //

            if (s4 > 0)
            {
                for (j = i; j < track.pointsEdgeRight.size(); j++)
                {
                    if (track.pointsEdgeRight[j].y < COLSIMAGE - 3 &&track.pointsEdgeRight[j].y < track.pointsEdgeRight[j -5].y)// track.pointsEdgeRight[j].slope > 0 

                       {
 s5 += 1;
 //printf("\n\n --------= s5=\t%d\t \n", s5);


                       }
                    else
                        s5 = 0;
                    if (s5 > 5)
                    {
                        printf("Right ring= s5=\t%d\t \n", s5);

                        break;
                    }
                }
            }
            if (s5 > 0)
            {
 printf("\n\n R----ight ring= s5=\t%d   j=%d\t \n", s5,j);
                for (i = j; i < track.pointsEdgeRight.size(); i++)
                {
                    // cout<<"s1=11111111111"<<endl;
                    if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i -5].y && track.pointsEdgeRight[i].slope < 0)
                       {
                         s6 += 1;
                          printf("\n\n -----Right ring= s6=\t%d\t \n", s6);
                       }
                    else
                        s6 = 0;
                    if (s6 >3)
                    {
                        printf("Right ring= s6=\t%d\t \n", s6);

                        break;
                    }
                }
            }
            if (s6 > 0)
            {

 printf("\n\n R----ight ring= s6=\t%d   i=%d\t \n", s6,i);

                for (j = i; j < track.pointsEdgeRight.size(); j++)
                {
                    // cout<<"s1=11111111111"<<endl;

                    // if (track.pointsEdgeRight[j].y > track.pointsEdgeRight[j - 2].y)
                    if (track.pointsEdgeRight[j].y > COLSIMAGE - 3 && track.pointsEdgeRight[j-1].y > COLSIMAGE - 3)
                       {
 s7 += 1;
  printf("----Right ring= s7=\t%d\t \n", s7);
                       }
                    else
                        s7 = 0;
                    if (s7 > 5)
                    {

                        printf("Right ring= s7=\t%d\t \n", s7);

                       
                       //
                        ringType = RingType::RingRight;
                        break;
                    }
                }
            }
        }
        // printf("\n ring-ringType=%d--logo=%d \t  s4=%d  s5=%d\t  s6=%d  s7=%d\n\n",ringType, logo,s4,s5,s6, s7);
        ///////////////////-----  左单边

        s4 = 0;
        s5 = 0;
        s8 = 0;

        if (logo == 1) //> 0 && ringType == RingType::RingLeft
        {
            // printf("\n\n\n  lvbo left 1\t\t %d\n", track.pointsEdgeLeft.size());

            //              for (int i = 0 ; i < track.pointsEdgeLeft.size() -1; i++)
            //         {

            //             printf("LEFT ring=%d=\t= %d=\t%d\t \n", i, track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y);
            //         }

            for (i = 2; i < track.pointsEdgeLeft.size(); i++)
            {
                // cout<<"s1=11111111111"<<endl;

                if (track.pointsEdgeLeft[i].y < 3 && track.pointsEdgeLeft[i - 1].y < 3)

                    s4 += 1;
                else
                    s4 = 0;
                if (s4 > 80)
                    break;
            }

            if (s4 > 0)
            {

                for (j = i; j < track.pointsEdgeLeft.size(); j++)
                {
                    // cout<<"s1=11111111111"<<endl;

                    if (track.pointsEdgeLeft[j].y > 2 && track.pointsEdgeLeft[j].y > track.pointsEdgeLeft[j - 2].y && track.pointsEdgeLeft[j].slope < -1)

                    {
                        s5 += 1;
                    }
                    else
                        s5 = 0;
                    if (s5 > 5)
                        break;
                }
            }

            if (s5 > 0)
            {
                for (i = j; i < track.pointsEdgeLeft.size(); i++)
                { //(track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 2].y)
                    if (track.pointsEdgeLeft[i].y < 3 && track.pointsEdgeLeft[i - 1].y < 3)
                        s8 += 1;
                    else
                        s8 = 0;
                    if (s8 > 15)
                    {
                        ringType = RingType::RingLeft;
                        break;
                    }
                }

                if (s8 < 1)
                {
                    for (i = j; i < (track.pointsEdgeLeft.size() - 10); i++)
                    {
                        if (track.pointsEdgeLeft.size() > 50 && track.pointsEdgeLeft[i].y < 3 && track.pointsEdgeLeft[i].slope > 3 && track.pointsEdgeLeft[i + 2].y < 3 && track.pointsEdgeLeft[i + 4].y < 3)
                        {
                            s8 = 1;
                            ringType = RingType::RingLeft;
                            break;
                        }
                    }
                }
            }
        }

        // cout << "----s1=" << s1 << "\t s2=" << s2 << "\t s3=" << s3 << endl;
        // cout << "----s4=" << s4 << "\t s5=" << s5 << "\t s6=" << s6 << endl;
        printf("\n ring-ringType=%d--logo=%d \t  s7=%d  s8=%d\n\n", ringType, logo, s7, s8);

        // vector<POINT> vec= track.pointsEdgeLeft;
        // vec.erase(vec.begin(), vec.begin() + track.validRowsLeft);
        // printf("\t ring--sigma(pointsEdgeLeft)=%3.2f \n\n",track.stdevEdgeCal(vec, ROWSIMAGE));   //sigma(vec)
        // printf("\t ring--track.validRowsRight=%d \n\n",track.validRowsRight);

        // vec= track.pointsEdgeRight;
        // vec.erase(vec.begin(), vec.begin() + track.validRowsRight);

        // for (int i = 0 ; i <vecr.size() -1; i++)
        // {
        //     printf("vecr ring=%d=\t= %d=\t%d\t \n", i, vecr[i].x, vecr[i].y);
        // }

        // printf("\t ring--sigma(pointsEdgeRight)=%3.2f \n\n",track.stdevEdgeCal(vec, ROWSIMAGE));
        //    s4>track.pointsEdgeLeft.size()-5)
        if (s7 > 0 || s8 > 0)
            // if(s3>0&&s6||s3>0&&averagepoint(pointsEdgeRight)<20||s6>0&&averagepoint(pointsEdgeLeft)<20)
            return true;
        else
            return false;
    }
};
