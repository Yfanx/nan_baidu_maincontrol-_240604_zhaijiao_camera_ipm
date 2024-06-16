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

   int pathstype = pathstype::None;


    int controlCenter=COLSIMAGE / 2;           // 智能车控制中心（0~320）

    int ipm_controlCenter= COLSIMAGE/2;          // 智能车控制中心（0~320）


    vector<POINT> centerEdge;     // 赛道中心点集
    uint16_t validRowsLeft = 0;   // 边缘有效行数（左）
    uint16_t validRowsRight = 0;  // 边缘有效行数（右）
    double sigmaCenter = 0;       // 中心点集的方差
    double jiaodu = 0;            // 中心点集的方差
    uint16_t validRowscenter = 0; // 边缘有效行数（右）
    /**
     * @brief 控制中心计算
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */

    void controlCenterCal(TrackRecognition &track,Edgeipm &edgeipm)
    {
        sigmaCenter = 0;
        controlCenter = COLSIMAGE / 2;
        centerEdge.clear();
        vector<POINT> v_center(4); // 三阶贝塞尔曲线edgeipm.ipm_controlCenter
       // style = "STRIGHT";

        int danbian = 0; // 左右单边标志 1--左 2---右  0--直

        float heightoffset = 0.5;
        track.validRowsCal();
printf("---66---Right-track----Left--Right.size()= %d   =%d  \n \n",track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());
        // printf("-1-concel---validRowsLeft=r--%d-----%d\n", track.validRowsLeft, track.validRowsLeftend);

	pathtypecheck(track);

		printf("\n\n control-pathtypecheck-  pathstype   =%d\n",pathstype);


track=edgeipm.ipm_pathstype_world(track, track.imageoriginal, pathstype);
        //


ipm_controlCenter=edgeipm.ipm_controlCenter;


         printf("-XXXX-concel---validRowsLeft=r--%d-----%d\n", track.validRowsLeft, track.validRowsLeftend);

      printf("-XXXX-concel---validRowsRight= r---%d   --%d\n", track.validRowsRight, track.validRowsRightend);
        //    for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        //         {
        //             printf("\n\n-0-con i=%d --track.pointsEdgeLeft[i].x=%d  \t  y=%d\n",i,track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y); // 绿色点

        //         }

         if (track.pointsEdgeLeft.size()+track.pointsEdgeRight.size()<10)
         {

            return ;
         } 
    printf("-rrr-concel---validRowsRight= r---%d   --%d\n", track.validRowsRight, track.validRowsRightend);
        // 左单边
        if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() <= 4 || (track.pointsEdgeLeft[track.validRowsLeft].x - track.pointsEdgeRight[track.validRowsRight].x) > 1.5 * validRowscenter)
        {
            if (track.validRowsLeftend - track.validRowsLeft > 5)
                track.pointsEdgeLeft.assign(track.pointsEdgeLeft.begin() + track.validRowsLeft, track.pointsEdgeLeft.begin() + track.validRowsLeftend - 2);
            else
                track.pointsEdgeLeft.resize(2); //.clear();

            if (track.pointsEdgeLeft.size() > validRowscenter)
            {
                track.pointsEdgeLeft.resize(validRowscenter);
            }

            style = "LEFT";
            //
            centerEdge = centerCompute(track.pointsEdgeLeft, 0);
        }
        // 右单边
        else if (track.pointsEdgeRight.size() > 4 && track.pointsEdgeLeft.size() <= 4 || (track.pointsEdgeRight[track.validRowsRight].x - track.pointsEdgeLeft[track.validRowsLeft].x) > 1.5 * validRowscenter) // ROWSIMAGE / 2
        {

            if (track.validRowsRightend - track.validRowsRight > 5)
            {
                track.pointsEdgeRight.assign(track.pointsEdgeRight.begin() + track.validRowsRight, track.pointsEdgeRight.begin() + track.validRowsRightend - 2);
            }
            else
                track.pointsEdgeRight.resize(2);

            if (track.pointsEdgeRight.size() > validRowscenter)
            {
                track.pointsEdgeRight.resize(validRowscenter);
            }
            //////////

            track.validRowsCal();
            track = retrack(track);
            // printf("-1-concel---validRowsRight= r---%d   --%d\n", track.validRowsRight, track.validRowsRightend);

            //////////////

            style = "RIGHT";
            //
            centerEdge = centerCompute(track.pointsEdgeRight, 1);
        }
        else
        {

            track = retrack(track);

            if (track.pointsEdgeRight.size() > 10 && track.pointsEdgeLeft.size() > 10)
            {
                if ((track.pointsEdgeRight.front().x - track.pointsEdgeLeft.front().x) > heightoffset * validRowscenter && track.pointsEdgeLeft.front().x - track.pointsEdgeRight.back().x > 10)

                    track.pointsEdgeRight.erase(track.pointsEdgeRight.begin(), track.pointsEdgeRight.end() - (track.pointsEdgeLeft.front().x - track.pointsEdgeRight.back().x));

                if (((track.pointsEdgeLeft.front().x - track.pointsEdgeRight.front().x) > heightoffset * validRowscenter) && track.pointsEdgeRight.front().x - track.pointsEdgeLeft.back().x > 10)
                {

                    // printf("\n control--Left.  right--front().x=%d --%d \n\n",track.pointsEdgeLeft.front().x,track.pointsEdgeRight.front().x);

                    track.pointsEdgeLeft.erase(track.pointsEdgeLeft.begin(), track.pointsEdgeLeft.end() - (track.pointsEdgeRight.front().x - track.pointsEdgeLeft.back().x));
                }
            }

            if (track.pointsEdgeLeft.size() > validRowscenter)
            {
                track.pointsEdgeLeft.resize(validRowscenter);
            }

            if (track.pointsEdgeRight.size() > validRowscenter)
            {
                track.pointsEdgeRight.resize(validRowscenter);
            }

            track.validRowsCal();
            track = retrack(track);
            //   控制行距离压缩 240116
            ////----------------------------广角进入十字前，车走斜时,砍行120，处理 1075 pic,2023-09-25
            /////////////////////  -----  右单边

            if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() > 4 && abs(track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x) < 0.5 * validRowscenter) // 通过双边缘有效点的差来判断赛道类型
            {
                // printf("\n\n-1-con i=ss\n");

                // for (int i = 0; i < 4; i++)
                // {
                //    printf("-1-con i=%d --v_center[].x=%d  \t  y=%d\n",i,v_center[i].x, v_center[i].y); // 绿色点

                // }
                style = "STRIGHT";

                centerEdge = centerComputestraight(track);
            }
            // 左单边
            else if ((track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() <= 4) ||
                     (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() > 0 && ((track.pointsEdgeLeft[0].x - track.pointsEdgeRight.back().x) > 0.5 * validRowscenter && 0.8 * track.pointsEdgeLeft.size() > track.pointsEdgeRight.size() || (track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x) > 0.5 * validRowscenter)))
            {
                printf("-1-con left\n");
                style = "LEFT";
                centerEdge = centerCompute(track.pointsEdgeLeft, 0);
            }
            // 右单边
            else if ((track.pointsEdgeRight.size() > 4 && track.pointsEdgeLeft.size() <= 4) || track.pointsEdgeLeft.size() < 0.8 * track.pointsEdgeRight.size() &&
                                                                                                   ((track.pointsEdgeRight[0].x - track.pointsEdgeLeft.back().x) > 0.5 * validRowscenter || (track.pointsEdgeLeft[0].x - track.pointsEdgeRight[0].x) > 0.5 * validRowscenter)) // ROWSIMAGE / 2
            {

                printf("-1-con right\n");
                style = "RIGHT";
                centerEdge = centerCompute(track.pointsEdgeRight, 1);
            }
        }

       
  printf("-ssss-concel---validRowsRight= r---%d   --%d\n", track.validRowsRight, track.validRowsRightend);

        jiaodu = centerline_xielv(centerEdge);

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
    }

    
    
    
    int pathtypecheck(TrackRecognition &track)
    {
        pathstype = pathstype::None;
        float heightoffset = 0.5;
        int i, j, s1, k1;

        //     //////

        //       }

        //     ////////////////////------------pathstype == pathstype:: pathRightxie-----------------

        //     //////////////////-----------pathstype == pathstype::pathLeftedge-----------------------------

        // printf("track.validRowsLeft<0.7*track.pointsEdgeLeft.size()---------%d  %d\n",track.validRowsLeft,track.pointsEdgeLeft.size());

        // if ((track.validRowsLeft-track.validRowsRight<120||track.validRowsLeft < track.validRowsRight) && track.pointsEdgeLeft.size() > 50 && vecmin(track.pointsEdgeLeft, track.validRowsLeft) > 0 && track.validRowsLeft < 0.8* track.pointsEdgeLeft.size())

        if (averagepoint(track.pointsEdgeLeft) > 20 && averagepoint(track.pointsEdgeRight) > COLSIMAGE - 10||(track.validRowsRightend - track.validRowsRight)<(track.validRowsLeftend-track.validRowsLeft))
        {

            pathstype = pathstype::pathLeftedge_only;
            printf("\n\n\n  ok-----------------pathstype::pathLeftedge\n");
            return 0;
        }
        //     ////////////////////------------pathstype == pathstype::pathLeftedge-----------------

        //     //  for (j = 0; j < track.pointsEdgeRight.size(); j++)
        //     //       // cout<<"s1=11111111111"<<endl;
        //     //   printf(" track.pointsEdgeLeft ---------%d  %d   %d\n",track.pointsEdgeRight[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);

        //     ////////////////////---------- pathstype = pathstype::pathRightedge----------------
printf("\n\n -cccc--averagepoint(track.pointsEdgeLeft)=%d===%3.2f++++++%3.2f\n",track.pointsEdgeRight.size() ,averagepoint(track.pointsEdgeLeft),averagepoint(track.pointsEdgeRight) );

printf("\n\n -cccc--bool=%d",(averagepoint(track.pointsEdgeLeft)<10 &&track.pointsEdgeRight.size()>40&&averagepoint(track.pointsEdgeRight)<( COLSIMAGE-40)));


        if (averagepoint(track.pointsEdgeLeft)<10 &&track.pointsEdgeRight.size()>40&&averagepoint(track.pointsEdgeRight)<(COLSIMAGE-40)||(track.validRowsRightend - track.validRowsRight)>=(track.validRowsLeftend-track.validRowsLeft))
        {

            pathstype = pathstype::pathRightedge_only; //
 printf("\n\n\n  ok-----------------pathRightedge_only   \n");
            //     ////////////////////---------- pathstype = pathstype::pathRightedge----------------

            return 0;
            //////////////
        }
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
        string str="pathstype: " + formatDoble2String(pathstype, 1) ;
        putText(centerImage, str, Point(COLSIMAGE - 160, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型
        //      std:string str0 = std::to_string(servoPwm);
        //	str = "PWM: " + str0;
        //  	putText(centerImage, str, Point(COLSIMAGE -30,dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, :0, 255), 1);//pwm
        str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
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
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
        {
            if (pointsEdgeLeft[i].y >= 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }

        return 0;
    }

    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }

        return 0;
    }

    /**
     * @brief 赛道中心点计算：单边控制
     *
     * @param pointsEdge 赛道边缘点集
     * @param side 单边类型：左边0/右边1
     * @return vector<POINT>
     */
    vector<POINT> centerCompute(vector<POINT> pointsEdge, int side)
    {
        int step = 2;                    // 间隔尺度
        int offsetWidth = COLSIMAGE / 2; // 首行偏移量
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
                    // counter = 0;
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
