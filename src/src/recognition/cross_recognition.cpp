

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
 * @file cross_recognition.cpp
 * @author Leo ()
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-06-04
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：track_recognition.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"
// #include "../../include/predictor.hpp"

// using namespace cv;
using namespace std;

class CrossroadRecognition
{
public:
    /**
     * @brief 初始化
     *
     */
    int validRowsLeft = 0;            // 边缘有效行数（左）
    int validRowsRight = 0;           // 边缘有效行数（右）
    void reset(void)
    {
        // crossroadType = CrossroadType::None; // 十字道路类型

        crossStep = crossStep::None;
    }

    /**
     * @brief 十字道路识别与图像处理
     *
     * @param track 赛道识别结果
     * @param imagePath 输入图像
     */
    bool crossroadRecognition(TrackRecognition &track, Mat &imagePath)
    {
        bool repaired = false; // 十字识别与补线结果
                               //  crossroadType = CrossroadType::None; // 十字道路类型

        // crossStep = crossStep::None;
   //
   printf("cross-validRowsLeft--Right-----=%d -=%d-----\n", track.validRowsLeft,track.validRowsRight );

   //printf("track-validRowsLeft--Right-x----=%d -=%d-----\n",track.pointsEdgeLeft[track.validRowsLeft].x ,track.pointsEdgeRight[track.validRowsRight].x);
   //cout<<" ="<< <<endl;
   //cout<<"validRows="<<<<endl;
   validRowsLeft =track.validRowsLeft;            // 边缘有效行数（左）
   validRowsRight =track.validRowsRight;           // 边缘有效行数（右）

        pointBreakLU = POINT(0, 0);
        pointBreakLD = POINT(0, 0);
        pointBreakRU = POINT(0, 0);
        pointBreakRD = POINT(0, 0);
 uint16_t rowBreakLU = 0,rowBreakLD = 0, rowBreakRU = 0,rowBreakRD=0;
        pointBreakLU_huitu = POINT(0, 0);
        pointBreakRU_huitu = POINT(0, 0);

        uint16_t counterRec = 0;    // 计数器
        uint16_t counterLinear = 0; // 连续计数器
        uint16_t  _index = 0;
        POINT bx1;
        float k,b;
        // if (track.pointsEdgeRight.size() < ROWSIMAGE / 3 || track.pointsEdgeLeft.size() < ROWSIMAGE / 3) // 十字有效行限制
        //  return false;

        _index = 1;
        //----------------------------------------------------------------------------------------------------

        uint16_t counter = 0;
        uint16_t rowBreakRightDown, rowBreakLeftUp, rowBreakLeftDown, kline;

        int shizitype = 0;                                        //  1--正如十字   2--zuoxieru   3--zuoxieru
        
        ///////////////////////

        if(track.pointsEdgeRight.size() >160)
        {
        rowBreakRU = searchBreakRightUp(track.pointsEdgeRight);   // 右上拐点搜索
        rowBreakRD = searchBreakRightDown(track.pointsEdgeRight); // 右下拐点搜索
        pointBreakRU = track.pointsEdgeRight[rowBreakRU];
        pointBreakRD = track.pointsEdgeRight[rowBreakRD];
        }

          if(track.pointsEdgeLeft.size() >160)
        {
        rowBreakLU = searchBreakLeftUp(track.pointsEdgeLeft);     // 左上拐点搜索
        rowBreakLD = searchBreakLeftDown(track.pointsEdgeLeft);   // 左下拐点搜索
        pointBreakLU = track.pointsEdgeLeft[rowBreakLU];
        pointBreakLD = track.pointsEdgeLeft[rowBreakLD];
        }


  // 
    printf("\n xxxxx-rowBreakLD-------=%d -rowBreakRD=%d-----\n", rowBreakLD, rowBreakRD);
    printf("xxxxx-rowBreakLU-------=%d -rowBreakRU=%d-----\n", rowBreakLU, rowBreakRU);
    printf("OOOO--BEFORE----crossStep=%d--------------\n", crossStep);

        // for (int i =0; i <track.pointsEdgeRight.size()  ; i++)
        //    {printf("\n\n i=== %d \tx=%d\t y=%d\n",i,track.pointsEdgeRight[i].x,track.pointsEdgeRight[i].y);   }
        if ((crossStep == crossStep::None&&searchxieCrossroad(track, track.pointsEdgeLeft, track.pointsEdgeRight))&& (rowBreakLD >5 || rowBreakRD >5)) //&&(rowBreakRU||rowBreakRU)

            crossStep = crossStep::Entering;
        else if (crossStep == crossStep::Entering && (rowBreakLD<6&& rowBreakRD<6) && (rowBreakLU > 0 || rowBreakRU > 0))

            crossStep = crossStep::Inside;
        else if (crossStep == crossStep::Inside && (rowBreakLU == 0 && rowBreakRU == 0))
            crossStep = crossStep::None;
        ///////////////////////////

        printf("after---crossStep=%d--------------\n", crossStep);



        //[02] 右入十字处理
        //////////////////////////////////////////////
        //[03] 直入十字处理
        // printf("xxxxx--track.stdevLeft--------=%3.2f-----\n",track.stdevLeft);

        // printf("xxxxx-rowBreakLD-------=%d -rowBreakRD=%d-----\n", rowBreakLD, rowBreakRD);

        //
         //printf("xxxxx-rowBreakLU-------=%d -rowBreakRU=%d-----\n", rowBreakLU, rowBreakRU);
  
  
  
  if (crossStep==crossStep::Entering)
            {
                cout<<"zhirushizi--CrossroadStraight-- in put\n"<<endl;
                  if (pointBreakLD.x<200&&pointBreakLD.x>80)
                {  printf("====================pointBreakLD.=====111==================""开始布线操作\n");
                
                    ///////////////////LEFT EDGE BUXIAN

                    bx1 = track.pointsEdgeLeft[rowBreakLD-5];
                    k = 1.0*(pointBreakLD.y - bx1.y)/(pointBreakLD.x - bx1.x);
                    // int yy = k * (st-pointBreakLD.x) + pointBreakLD.y;
                    // pointBreakLU = POINT{st,yy};
                    // cout<<"ll-rowBreakRD"<<"\t="<<pointBreakLU.x<<"\t"<<pointBreakLU.y<<endl;
                    // rowBreakLU=track.pointsEdgeLeft.size();
                    ////////////////////////
                    b = pointBreakLD.y - k * pointBreakLD.x;
                    for (int i = rowBreakLD; i <track.pointsEdgeLeft.size(); i++)
                    {
       
                        track.pointsEdgeLeft[i].y =k * track.pointsEdgeLeft[i].x+ b;
                    } 
                    repaired = true; // 补线成功
                }
                else
                    if (rowBreakLU+13<track.pointsEdgeLeft.size()&&(pointBreakLU.x>100||(pointBreakLD.x>230&&rowBreakLU>1)))
                {
                   printf("====================pointBreakLD.=====222=================""开始布线操作\n");
                    ///////////////////LEFT EDGE BUXIAN
                    POINT bx1 = track.pointsEdgeLeft[rowBreakLU+13];
                    k = 1.0*(pointBreakLU.y - bx1.y)/(pointBreakLU.x - bx1.x);
                    ////////////////////////
                    b = pointBreakLU.y - k * pointBreakLU.x;
                    for (int i = rowBreakLU; i > MIN(track.pointsEdgeLeft.size(), 10); i--)
                    {
                        track.pointsEdgeLeft[i].y = k * track.pointsEdgeLeft[i].x + b;
                        if (track.pointsEdgeLeft[i].y <3)
                            break;
                    }                  
                }


                if (pointBreakRD.x<200&&pointBreakRD.x>100)
                {
                    bx1 = track.pointsEdgeRight[rowBreakRD - 5];
                    k = 1.0 * (pointBreakRD.y - bx1.y) / (pointBreakRD.x - bx1.x);
                    // yy = k * (1 - pointBreakRD.x) + pointBreakRD.y;
                    // pointBreakRU = POINT{1,yy};
                    // rowBreakRU=track.pointsEdgeRight.size();
                    // cout<<"RR-rowBreakRD"<<"\t="<<pointBreakRU.x<<"\t"<<pointBreakRU.y<<endl;
                    b = pointBreakRD.y - k * pointBreakRD.x;
                    for (int i = rowBreakRD; i <= track.pointsEdgeRight.size(); i++)
                    {
                        track.pointsEdgeRight[i].y = k * track.pointsEdgeRight[i].x + b;
                    }
                   
                }
                else
          if (track.spurroad.size()>0||pointBreakRD.x>199&&rowBreakRU>1||(rowBreakRD<1&&rowBreakRU>1))
                {
                  printf("====11======000000=========rowBreakRU======================""开始布线操作\n");
       //waitKey(1500);  


printf("--=track.spurroad[0].x=  %d---%d\n",track.spurroad[0].x,track.spurroad[0].y);
printf("--track.pointsEdgeRight[0].x=  %d---%d\n",track.pointsEdgeRight[0].x,track.pointsEdgeRight[0].y);


  //waitKey(1500);  

               if (track.spurroad[0].x-track.pointsEdgeRight[0].x<60&&track.spurroad[0].y-track.pointsEdgeRight[0].y>0&&track.spurroad[0].y-track.pointsEdgeRight[0].y<60)
                     bx1 =track.spurroad[0];
                     else
                   bx1 = track.pointsEdgeRight[rowBreakRU+13];//LEFT EDGE BUXIAN
                   k = 1.0*(pointBreakRU.y - bx1.y)/(pointBreakRU.x - bx1.x);
                   b = pointBreakRU.y - k * pointBreakRU.x;
                    for (int i = rowBreakRU; i > MIN(track.pointsEdgeRight.size(), 10); i--)
                    {
                        track.pointsEdgeRight[i].y = k * track.pointsEdgeRight[i].x + b;
                        if (track.pointsEdgeRight[i].y>COLSIMAGE-3)
                            break;
                    }   
                }

 repaired = true; // 补线成功
        // printf("--=rowBreakLU=X Y=  %d---%d\n",track.pointsEdgeLeft[rowBreakLU].x,track.pointsEdgeLeft[rowBreakLU].y);
        // printf("--=rowBreakRU=X Y=  %d---%d\n",track.pointsEdgeRight[rowBreakRU].x,track.pointsEdgeRight[rowBreakRU].y);
   }
         
        if (crossStep==crossStep::Inside)
            {  //cout<<"zhirushizi--CrossroadStraight-- in put\n"<<endl;
                if (rowBreakLU>5)
                {
                    cout<<"==========================================="<<"开始布线操作"<<endl;
                    ///////////////////LEFT EDGE BUXIAN
                    POINT bx1 = track.pointsEdgeLeft[rowBreakLU+13];
                    float k = 1.0*(pointBreakLU.y - bx1.y)/(pointBreakLU.x - bx1.x);
                    ////////////////////////
                    float b = pointBreakLU.y - k * pointBreakLU.x;

                    for (int i = rowBreakLU; i > MIN(track.pointsEdgeLeft.size(), 10); i--)
                    {
                        track.pointsEdgeLeft[i].y = k * track.pointsEdgeLeft[i].x + b;
                        if (track.pointsEdgeLeft[i].y <3)
                            break;
                    }
                    
                }


        // printf("--=rowBreakLU=X Y=  %d---%d\n",track.pointsEdgeLeft[rowBreakLU].x,track.pointsEdgeLeft[rowBreakLU].y);
        // printf("--=rowBreakRU=X Y=  %d---%d\n",track.pointsEdgeRight[rowBreakRU].x,track.pointsEdgeRight[rowBreakRU].y);
                if (rowBreakRU>5)
                {
                printf("===================rowBreakRU= 2=======================""开始布线操作\n");
                printf("--=track.spurroad[0].x=  %d---%d\n",track.spurroad[0].x,track.spurroad[0].y);
printf("--track.pointsEdgeRight[0].x=  %d---%d\n",track.pointsEdgeRight[rowBreakRU].x,track.pointsEdgeRight[rowBreakRU].y);
              
               if (track.spurroad[0].x-track.pointsEdgeRight[rowBreakRU].x<60&&track.spurroad[0].y-track.pointsEdgeRight[rowBreakRU].y>0&&track.spurroad[0].y-track.pointsEdgeRight[rowBreakRU].y<60)
                     bx1 =track.spurroad[0];
                     else
                   bx1 = track.pointsEdgeRight[rowBreakRU+13];//LEFT EDGE BUXIAN
                   k = 1.0*(pointBreakRU.y - bx1.y)/(pointBreakRU.x - bx1.x);
                   b = pointBreakRU.y - k * pointBreakRU.x;
                    for (int i = rowBreakRU; i > MIN(track.pointsEdgeRight.size(), 10); i--)
                    {
                        track.pointsEdgeRight[i].y = k * track.pointsEdgeRight[i].x + b;
                        if (track.pointsEdgeRight[i].y>COLSIMAGE-3)
                            break;
                    }   
                }

repaired = true; // 补线成功

        }
       
      ////////////////----------------------zuo xieru shizi rowBreakLD >5 &&

        ///////////////////////////////----------------------zuo xieru shizi
        // cout << "rowBreakLU"
        //      << "==" << rowBreakLU << "rowBreakLD"
        //      << "==" << rowBreakLD << "rowBreakRU"
        //      << "==" << rowBreakRU << "rowBreakRD"
        //      << "==" << rowBreakRD << endl;

        //repaired = true; // 补线成功

        //
        return repaired;
    }

    /**
     * @brief 绘制十字道路识别结果
     *
     * @param Image 需要叠加显示的图像/RGB
     */
    void drawImage(TrackRecognition track, Mat &Image)
    {
        // 绘制边缘点
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制岔路点
        // for (int i = 0; i < track.spurroad.size(); i++)
        // {
        //     circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6,
        //            Scalar(0, 0, 255), -1); // 红色点
        // }

        //////////////////////

        // cout<<"--BreakLU.x-y="<< pointBreakLU.x<<"\t"<< pointBreakLU.y<<endl;
        // cout<<"--pointBreakLD.x-y="<<pointBreakLD.x<<"\t"<< pointBreakLD.y<<endl;

        // cout<<"--BreakRU.x-y="<< pointBreakRU.x<<"\t"<< pointBreakRU.y<<endl;
        // cout<<"--pointBreakRD.x-y="<<pointBreakRD.x<<"\t"<< pointBreakRD.y<<endl;
        /////////////////////////////

        drawMarker(Image, Point(pointBreakLU.y, pointBreakLU.x), Scalar(0, 255, 0), MARKER_TRIANGLE_UP, 20, 15, 16);
        // drawMarker(Image, Point(pointBreakLU_huitu.y, pointBreakLU_huitu.x),Scalar(0,255,0),MARKER_TRIANGLE_UP, 20, 15, 16);

        drawMarker(Image, Point(pointBreakRU.y, pointBreakRU.x), Scalar(0, 255, 255), MARKER_TRIANGLE_UP, 20, 15, 16);
        // circle(Image, Point(pointBreakLU_huitu.y, pointBreakLU_huitu.x), 15, Scalar( 0, 0,255), -1); // 上补线点：紫色
        circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(0, 255, 0), 5); // 下补线点：粉色
        // circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(0, 0,255), 9); // 上补线点：紫色
        circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(0, 255, 255), 5); // 下补线点：粉色

        //putText(Image, "Straight", Point(COLSIMAGE / 2 - 20, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }
        putText(Image, "Cross", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型

        putText(Image, to_string(crossStep), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);
    }

private:
    int _index = 0; // 测试

    POINT pointBreakLU;
    POINT pointBreakLD;
    POINT pointBreakRU;
    POINT pointBreakRD;

    POINT pointBreakLU_huitu;
    POINT pointBreakRU_huitu;

    uint16_t counterFild = 0;

    /**
     * @brief 十字道路类型
     *
     */
    // enum CrossroadType
    // {
    //     None =0,
    //     CrossroadLeft,     // 左斜入十字
    //     CrossroadRight,    // 右斜入十字
    //     CrossroadStraight, // 直入十字
    // };

    // CrossroadType crossroadType = CrossroadType::None; // 十字道路类型

    enum crossStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
                  // Finish    // 环任务结束
    };

    // RingType ringType = RingType::RingLeft; // 环岛类型
    crossStep crossStep = crossStep::None; // 环岛处理阶段

    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = 0;
        uint16_t counter = 0;

        uint16_t i1 = 0, i0 = validRowsLeft, logo = 0;

        // for (int i = pointsEdgeLeft.size() - 5; i > 0; i--)

        i0 = validRowsLeft;

        //             printf("LEFT slope=%d=\t= %d=\t%d\t== %3.4f \n", i, pointsEdgeLeft[i].x, pointsEdgeLeft[i].y, pointsEdgeLeft[i].slope);
        //         }
        //
        //  printf("\n\n\n  LEFT up validRowsLeft ======\t\t %d\n",validRowsLeft );
        // for (int i = i0; i < pointsEdgeLeft.size() * 0.95; i++)
        // {

        //     printf("LEFT slope=%d=\t= %d=\t%d\t== %3.4f \n", i, pointsEdgeLeft[i].x, pointsEdgeLeft[i].y, pointsEdgeLeft[i].slope);
        // }

        if (validRowsLeft < 1 && pointsEdgeLeft[validRowsLeft].x > 230)
        {

            for (int i = validRowsLeft; i < pointsEdgeLeft.size() * 0.95; i++)
            {
                // if (pointsEdgeLeft[i].slope>-0.6&&pointsEdgeLeft[i+1].slope>-0.6&&pointsEdgeLeft[i].slope<-0.1&&pointsEdgeLeft[i+1].slope<-0.1);
                if ((i + 10) < pointsEdgeLeft.size() * 0.95 && pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i + 1].y < 2 && pointsEdgeLeft[i + 3].y < 2 && pointsEdgeLeft[i + 4].y < 2 && pointsEdgeLeft[i + 7].y > 2 && pointsEdgeLeft[i + 8].y > 2 && pointsEdgeLeft[i + 9].y > 2 && pointsEdgeLeft[i + 10].y > 2)
                // abs(pointsEdgeRight[i+4].y - pointsEdgeRight[i +5].y)<2&&abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) > 3)

                { // printf("\n\n\n  i+4- LEFT validRowsLeft====== %d \n",  i+4);
                    i1 = i + 4;
                    break;
                }
            }



    counter = 0;
        for (int i = i1; i < pointsEdgeLeft.size() * 0.95-10; i++)
        {
            // if (pointsEdgeLeft[i].slope>-0.6&&pointsEdgeLeft[i+1].slope>-0.6&&pointsEdgeLeft[i].slope<-0.1&&pointsEdgeLeft[i+1].slope<-0.1);
            if (pointsEdgeLeft[i + 5].y > 10 && pointsEdgeLeft[i].slope < -5.1 && pointsEdgeLeft[i + 1].slope < -5.1 && pointsEdgeLeft[i + 7].slope > -2 && pointsEdgeLeft[i + 4].slope > -2 && pointsEdgeLeft[i + 5].slope > -2 && pointsEdgeLeft[i + 6].slope > -2)
            // abs(pointsEdgeRight[i+4].y - pointsEdgeRight[i +5].y)<2&&abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) > 3)

            {
                logo = 1;
                rowBreakLeftUp = i +8;
                break;
            }
        }
        }

 //////
        if (logo == 0)
        {
            for (int i = 0; i < pointsEdgeLeft.size() * 0.9; i++) // 寻找左边跳变点
            {

                if ((i + 8) < pointsEdgeLeft.size() * 0.95 && pointsEdgeLeft[i + 4].x<200 &&pointsEdgeLeft[i].y<2&& pointsEdgeLeft[i + 1].y<2&& pointsEdgeLeft[i].slope >= 0 && pointsEdgeLeft[i + 1].slope>=0 && pointsEdgeLeft[i + 2].slope>= 0 && pointsEdgeLeft[i + 4].slope<0 && pointsEdgeLeft[i + 5].slope< 0 && pointsEdgeLeft[i + 6].slope<0)
                {
                    logo = 1;
                    //
                    rowBreakLeftUp = i +4;
                    printf("\n\n\n  search=rowBreakLeftDown====== %d \t\t x= %d \t y=%d\n", rowBreakLeftUp, pointsEdgeLeft[i +4].x, pointsEdgeLeft[i +4].y);

                    break;
                    // return rowBreakRightUp;
                }
            }
        }

        // if ( logo <1&&i0 >0&&pointsEdgeLeft[i0].x>20)
        /////////
     return rowBreakLeftUp;

        // if (logo > 0 && rowBreakLeftUp > 3 && (i0 > 1 || i0 < 1 && (pointsEdgeLeft[i1].x - pointsEdgeLeft[rowBreakLeftUp].x) < 45))
        //     return rowBreakLeftUp;
        // else
        //     return 0;
    }

    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    { //printf("\n\n\n-- ssss-- search=rowBreakLeftDown====== \n");
        uint16_t rowBreakLeftDown = 0;
        uint16_t i0 = 0, logo = 0, counter = 0;

        for (int i = pointsEdgeLeft.size() * 0.9; i > 0; i--) // 寻找左边跳变点
        {

            if (pointsEdgeLeft[i].y < 3)
            {

                counter++;
            }
            else
                counter = 0;
            if (counter > 5)

            {

                i0 = i;
                break;
            }
        }

        counter = 0;
        if (i0 > 0)
        {
            for (int i = 0; i < i0; i++) // 寻找左边跳变点
            {

                if (pointsEdgeLeft[i].y <= pointsEdgeLeft[i + 1].y && pointsEdgeLeft[i].y > 5)
                    counter++;
                else
                    counter = 0;

                if (counter > 5 && pointsEdgeLeft[i + 1].y > pointsEdgeLeft[i + 2].y && (pointsEdgeLeft[i + 1].y - pointsEdgeLeft[i + 2].y) > 2)
                {
                    logo = 1;
                    rowBreakLeftDown = i;
                    break;
                }
            }
        }

//printf("\n\n\n-- eeeee-- search=rowBreakLeftDown====== \n");


        //////
        if (logo == 0)
        {  printf("\n\n\n-- 0000-- search=rowBreakLeftDown====== \n");

            for (int i = 0; i < pointsEdgeLeft.size() * 0.9; i++) // 寻找左边跳变点
            {

                if (i>10&&(i +10) < pointsEdgeLeft.size() * 0.95 && pointsEdgeLeft[i].x >80 && pointsEdgeLeft[i].slope < 0 && pointsEdgeLeft[i -5].slope<0 && pointsEdgeLeft[i -6].slope<0 && pointsEdgeLeft[i -7].slope<0 && pointsEdgeLeft[i + 5].slope >0 && pointsEdgeLeft[i + 6].slope>0 && pointsEdgeLeft[i + 4].slope >= 0)
                {
                    logo = 1;
                    //
                    rowBreakLeftDown= i+4;
                    printf("\n\n\n ----rrr---- search=rowBreakLeftDown====== %d \t\t x= %d \t y=%d\n", rowBreakLeftDown, pointsEdgeLeft[i + 2].x, pointsEdgeLeft[i + 2].y);
             
                    break;
                    // return rowBreakRightUp;
                }
            }
        }

        /////////// 左斜入十字时
        if (logo == 1 && rowBreakLeftDown > 8 && rowBreakLeftDown < pointsEdgeLeft.size() * 0.9 && pointsEdgeLeft[rowBreakLeftDown - 4].x >80)
            return rowBreakLeftDown -4;
        else
            {
                
                 printf("\n\n\n ----rrr----not found leftdown jiaodian=====");
                
                return 0;


            }
        ///////////////////////////
    }

    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightUp = 0;
        uint16_t i0 = 0, i1 = validRowsRight, counter = 0;

        // printf("\n\n\n searchBreakRightUp ------------、=%d \n\n", validRowsRight);
       
         if (validRowsRight<80&&pointsEdgeRight[validRowsRight].x>180)
        {

            for (int i = validRowsRight; i < pointsEdgeRight.size() * 0.95; i++)
            {
                // if (pointsEdgeRight[i].slope>-0.6&&pointsEdgeRight[i+1].slope>-0.6&&pointsEdgeRight[i].slope<-0.1&&pointsEdgeRight[i+1].slope<-0.1);
                if ((i + 10) < pointsEdgeRight.size() * 0.95 && pointsEdgeRight[i].y > COLSIMAGE - 5 && pointsEdgeRight[i + 1].y > COLSIMAGE - 5 && pointsEdgeRight[i + 3].y > COLSIMAGE - 5 && pointsEdgeRight[i + 4].y > COLSIMAGE - 5 && pointsEdgeRight[i + 7].y < COLSIMAGE - 5 && pointsEdgeRight[i + 8].y < COLSIMAGE - 5 && pointsEdgeRight[i + 9].y < COLSIMAGE - 5 && pointsEdgeRight[i + 10].y < COLSIMAGE - 5)
                // abs(pointsEdgeRight[i+4].y - pointsEdgeRight[i +5].y)<2&&abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) > 3)

                {
                    printf("\n\n\n searchBreakRightUp i+4- Right validRowsRight====== %d-----y=%d \n", i + 4, pointsEdgeRight[i + 4].y);
                    i1 = i + 4;
                    break;
                }
            }
        }
        else
        i1 = validRowsRight;

        //  printf("\n\n\n  right up io====== %d \t\t %d\n",validRowsRight,pointsEdgeRight.size());
        //       for (int i = validRowsRight; i < pointsEdgeRight.size() * 0.95; i++)
        //         {

        //  printf("right slope=%d== %d=\t%d\t== %3.4f \n",i,pointsEdgeRight[i].x,pointsEdgeRight[i].y,pointsEdgeRight[i].slope);
        //         }

        for (int i = i1; i < pointsEdgeRight.size() * 0.95; i++)
        {

            if ((i + 7) < pointsEdgeRight.size() * 0.95 && pointsEdgeRight[i].slope > 3 && pointsEdgeRight[i + 1].slope > 3 && pointsEdgeRight[i + 6].slope < 2 && pointsEdgeRight[i + 7].slope < 2) //&&pointsEdgeRight[i+2].slope>0&&pointsEdgeRight[i+3].slope>0
            {                                                                                                                                                                                        // printf("\n\n\n  999==right up io====== %d \t\tpointsEdgeRight[i].slope= %d \t i+1=%d\n",i0,pointsEdgeRight[i].slope,pointsEdgeRight[i+1].slope);
                rowBreakRightUp = i + 5;
                break;
                // return rowBreakRightUp;
            }
        }

        if (rowBreakRightUp > 3 && pointsEdgeRight[rowBreakRightUp].x > 50 && ((pointsEdgeRight[i1].x - pointsEdgeRight[rowBreakRightUp].x) < 35))
            return rowBreakRightUp;
        else
            return 0;

        return rowBreakRightUp;
    }

    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightDown = 0;
        uint16_t counter = 0;
        bool start = false;
        uint16_t i0 = 0, logo = 0;

        // printf("\n\n\n  right up io====== %d \t\t %d\n", i0, pointsEdgeRight.size());
        // for (int i = validRowsRight; i < pointsEdgeRight.size() * 0.65; i++)
        // {

        //     printf("right slope=%d== %d=\t%d\t== %3.4f \n", i, pointsEdgeRight[i].x, pointsEdgeRight[i].y, pointsEdgeRight[i].slope);
        // }
        //////////////////////////////////////

        // for (int i = pointsEdgeRight.size() * 0.9; i > 0; i--) // 寻找左边跳变点
        // {
        //     if (pointsEdgeRight[i].y > COLSIMAGE - 5)
        //     {
        //         counter++;
        //     }
        //     else
        //         counter = 0;
        //     if (counter > 15)

        //     {
        //         i0 = i;
        //         break;
        //     }
        // }

        ////////////////

        // counter = 0;
        // if (i0 > 0)
        // {
        //     for (int i = 0; i < i0; i++) // 寻找左边跳变点
        //     {

        //         if (pointsEdgeRight[i].y < COLSIMAGE - 10 && pointsEdgeRight[i].y >= pointsEdgeRight[i + 1].y)
        //             counter++;
        //         else
        //             counter = 0;
        //         if ((counter > 5 && pointsEdgeRight[i + 1].y < pointsEdgeRight[i + 2].y && (pointsEdgeRight[i + 2].y - pointsEdgeRight[i + 1].y) > 2) || (pointsEdgeRight[i].slope > 0.1 && pointsEdgeRight[i + 1].slope > 0.1 && pointsEdgeRight[i + 2].slope < 0 && pointsEdgeRight[i + 3].slope < 0))
        //         {
        //             logo = 1;
        //             rowBreakRightDown = i;
        //             break;
        //         }
        //     }
        // }

        //////////

        if (logo == 0)
        {

            for (int i = 0; i < pointsEdgeRight.size() * 0.95; i++)
            {

                if ((i + 8) < pointsEdgeRight.size() * 0.95 && pointsEdgeRight[i + 4].x > 70 && pointsEdgeRight[i].slope > 0 && pointsEdgeRight[i + 1].slope > 0 && pointsEdgeRight[i + 2].slope > 0 && pointsEdgeRight[i + 7].slope < 0 && pointsEdgeRight[i + 4].slope <=0 && pointsEdgeRight[i + 5].slope <=0 && pointsEdgeRight[i + 6].slope < 0)
                {
                    logo = 1; 
                    // 
                     rowBreakRightDown = i + 2;
                     printf("\n\n\n  search=rowBreakRightDown====== %d \t\t x= %d \t y=%d\n",rowBreakRightDown,pointsEdgeRight[i+2].x,pointsEdgeRight[i+2].y);
                  
                    break;
                    // return rowBreakRightUp;
                }
            }
        }

        ///////////
        if (logo == 1 && rowBreakRightDown>5 && rowBreakRightDown< pointsEdgeRight.size() * 0.7&&pointsEdgeRight[rowBreakRightDown].x > 100)
            return rowBreakRightDown;
        else
            return 0;
    }

    bool searchStraightCrossroad(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        if (pointsEdgeLeft.size() < ROWSIMAGE * 0.8 || pointsEdgeRight.size() < ROWSIMAGE * 0.8)
        {
            return false;
        }

        uint16_t counterLeft = 0;
        uint16_t counterRight = 0;
        for (int i = pointsEdgeLeft.size() - 10; i > 1; i--) // 搜索上半部分边缘点
        {
            if (pointsEdgeLeft[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeLeft[i].y < 2)
                counterLeft++;
        }
        for (int i = pointsEdgeRight.size() - 10; i > 1; i--) // 搜索上半部分边缘点
        {
            if (pointsEdgeRight[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                counterRight++;
        }
        if (counterLeft > 30 && counterRight > 30)
            return true;
        else
            return false;
    }

    bool searchxieCrossroad(TrackRecognition&track,vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        // track.stdevLeft<100||track.stdevRight<100||
        if (pointsEdgeLeft.size() < ROWSIMAGE * 0.9 || pointsEdgeRight.size() < ROWSIMAGE * 0.9)
        {
            return false;
        }
        int widthyuzhi=0;
        printf("----000--searchxieCrossroad--%d==--validRowsLeft=%d\t  \n",pointsEdgeLeft[validRowsLeft].x,validRowsLeft);

//////////////////////////////////
    uint16_t i = 0, j = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0, cnt = 0;
        //
//if (track.pointsEdgeLeft.size() > ROWSIMAGE * 0.9) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
{
    for (j =0; j < track.pointsEdgeLeft.size() - 5; j++)
    {                                                                  // track.pointsEdgeLeft[j].y>100&&track.widthBlock[j +3].y<180&&
        if ((track.widthBlock[j].y - track.widthBlock[j + 2].y) >100) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
        {
            widthyuzhi = 100;
            break;
        }
    }
}

///////////////////////-----------------------右邊十字



///////////////////////-----------------------右邊十字

//for (int i =0; i < pointsEdgeLeft.size()-5; i++)
//printf("-track.widthBlock[j].y--%d=%d\t  \n",pointsEdgeLeft[i].x,track.widthBlock[i].y - track.widthBlock[i+ 2].y);


if (averagepoint(pointsEdgeRight) > COLSIMAGE - 20||averagepoint(pointsEdgeLeft) <20)
{
    for (j = 0; j < track.pointsEdgeLeft.size() - 5; j++)
    {                                                            // track.pointsEdgeLeft[j].y>100&&track.widthBlock[j +3].y<180&&
        if ((track.widthBlock[j].y > track.widthBlock[j + 5].y)) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
        {
            s1 = s1 + 1; 
        }
            if (s1 > 20 && track.widthBlock[j].y < track.widthBlock[j + 5].y)
                {
                    //printf("--ttttt--s1=%d\t  \n",s1);               
                    break;
                }     
    }
    // track.pointsEdgeLeft[j].y>100&&track.widthBlock[j +3].y<180&&
    if (s1> 20 && j < 0.8 * ROWSIMAGE) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
    {
//printf("----hhhh--searchxieCrossroad--%d==--validRowsLeft=%d\t  \n",pointsEdgeRight.size(),validRowsLeft);

        for (i = j; i < track.pointsEdgeLeft.size() - 10; i++)
        {
            if ((track.widthBlock[i].y < track.widthBlock[i + 5].y)) // &&track.pointsEdge[j+1].y<(COLSIMAGE-2)&& track.pointsEdge[j+1].y>track.pointsEdge[j+2].y&&track.pointsEdge[j + 1].slope>4&& track.pointsEdge[j+2].slope>4&&track.pointsEdge[j +5].slope<1&&track.pointsEdge[j +6].slope<1)
            {
                s2++; 
                //printf("--kk--s2=%d\t  \n",s2);
            }
                if (s2 > 20 && track.widthBlock[i + 5].y > COLSIMAGE - 5)
                    {//printf("--ttttt--s2=%d\t  \n",s2);                    
                    return true;
                    }          
        }
    }
}


//   if (widthyuzhi <1)
//         {
//             return false;
//         }

  printf("---111---searchxieCrossroad--%d==--validRowsLeft=%d\t  \n",pointsEdgeLeft[validRowsLeft].x,validRowsLeft);

  uint8_t logo = 0;
    s1 = 0, s2 = 0;
        /////////////////////  -----  右单边
        //   for (i =validRowsLeft; i < pointsEdgeLeft.size(); i++)
        //           printf("-s0-x,y--%d=%d\t  \n",pointsEdgeLeft[i].x,pointsEdgeLeft[i].y);

        // if (pointsEdgeLeft.size()>150)
        // cout<<"s1=0000000000"<<endl;
        for (i = validRowsLeft; i < pointsEdgeLeft.size(); i++)
        { // cout<<"s1=11111111111"<<endl;
            if (i-2>0&&pointsEdgeLeft[i].y>pointsEdgeLeft[i-2].y)
            {
                // printf("--i--s1=%d\t  \n",i);
                s1 += 1;
            }
            else
                s1 = 0;
            if (s1 > 3)
                break;
        }

        if (s1 > 0)
        {
            // for (j = i; j < pointsEdgeLeft.size(); j++)
            // printf("-s2-x,y--%d=%d\t  \n",pointsEdgeLeft[j].x,pointsEdgeLeft[j].y);
            for (j = i; j < pointsEdgeLeft.size(); j++)
            { // cout<<"s1=11111111111"<<endl;
                if (pointsEdgeLeft[j].y < 3 && pointsEdgeLeft[j - 1].y < 3)
                {
                    s2 += 1;
                }
                else
                    s2 = 0;
                if (s2 > 10)
                    break;
            }
        }

        if (s2 > 0)
        {
            //    for (i = j; i < pointsEdgeLeft.size(); i++)

            //          printf("--x,y--%d=%d\t  \n",pointsEdgeLeft[i].x,pointsEdgeLeft[i].y);

            for (i = j; i < pointsEdgeLeft.size(); i++)
            {
                if (pointsEdgeLeft[i].y > 15 && pointsEdgeLeft[i].y >= pointsEdgeLeft[i - 2].y)
                {
                    printf("--i--s3=%d\t  \n", i);
                    s3 += 1;
                }
                else
                    s3 = 0;
                if (s3 > 3)
                    break;
            }
        }

        ///////////////////-----  右单边
        //////////////////-----  左单边
        // 边缘有效行优化
        // if (pointsEdgeRight.size()>150)

        for (i = 2; i < pointsEdgeRight.size(); i = i + 1)
        { // cout<<"s1=11111111111"<<endl;
            if (pointsEdgeRight[i].y < pointsEdgeRight[i - 2].y)
                s4 += 1;
            else
                s4 = 0;
            if (s4 > 5)
                break;
        }

        if (s4 > 5)
        {
            for (j = i; j < pointsEdgeRight.size(); j++)
            { // cout<<"s1=11111111111"<<endl;

                if (pointsEdgeRight[j].y > COLSIMAGE - 3 && pointsEdgeRight[j - 1].y > COLSIMAGE - 3)
                {
                    s5 += 1;
                }
                else
                    s5 = 0;
                if (s5 > 15)
                    break;
            }
        }
        if (s5 > 15)
        {
            for (i = j; i < pointsEdgeRight.size(); i = i + 1)
            {
                if (pointsEdgeRight[i].y < pointsEdgeRight[i - 2].y)
                    s6 += 1;
                else
                    s6 = 0;
                if (s6 > 5)
                    break;
            }
        }
        ///////////////////-----  左单边
        // printf("----s1=%d\t s2=%d \n", s1, s2);
        // printf("----s4=%d\t s5=%d \n", s4, s5);
        // // cout<<"----s4="<<s4<<"\t s5="<<s5<<"\t s6="<<s6<<endl;
        // //
        // printf("\tcross--- s3=%d \t s6=%d\n", s3, s6);
        //   s4>track.pointsEdgeLeft.size()-5)
        // if((s1>3&&s2>0&&s3>0&&s4>0&&s5>0&&s6>0)||(s1>3&&s2>20&&s3>15&&s5>100)||(s4>5&&s5>20&&s6>5&&s2>100))
        if (s3 > 0 && s6 > 0 || s3 > 0 && averagepoint(pointsEdgeRight) < 20 || s6 > 0 && averagepoint(pointsEdgeLeft) < 20)
            logo = 1;
        else
        {
            s1 = searchBreakRightDown(pointsEdgeRight); // 右下拐点搜索
            printf("\n\n\n   --xieru--searchBreakRightDown---=%d \n", s1 );
            if (s1 > 0 && pointsEdgeLeft[validRowsLeft].x<150 && validRowsLeft < (pointsEdgeLeft.size() - 4)) //&& (pointsEdgeLeft[validRowsLeft + 1].y - pointsEdgeLeft[validRowsLeft].y > 120)
                logo = 1;
            else
            {
                s2 = searchBreakLeftDown(pointsEdgeLeft); // 左下拐点搜索
                if (s2 > 0 && pointsEdgeRight[validRowsRight].x<150 && validRowsRight < (pointsEdgeRight.size() - 4))// && (pointsEdgeRight[validRowsRight ].y - pointsEdgeRight[validRowsRight+ 1].y > 120))
                    logo = 1;
                // pointBreakRU = track.pointsEdgeRight[rowBreakRU];
            }
        }
printf("----logo---=%d \n", logo);
if (logo==1)
    return true;
else
    return false;
}

};
