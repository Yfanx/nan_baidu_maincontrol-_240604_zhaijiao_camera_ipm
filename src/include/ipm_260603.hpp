#pragma once

// #include <vector>
#include "../src/image_preprocess.cpp"
#include "../src/perspective_mapping.cpp"
#include "../src/recognition/track_recognition.cpp" //赛道识别基础类
#include "./common.hpp"                             //公共类方法文件
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

class Edgeipm
{
private:
  int counterShift = 0; // 变速计数器
public:
  vector<POINT> ipmpointsEdgeLeft, ipmpointsEdgeRight;

  Mat frame0;

  pathstype ipm_pathstype = pathstype::None;
  vector<POINT> pointsEdgeLeftnew;  // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew; // 赛道右边缘点集
  vector<POINT> centerEdge, centerEdgereal;
  int laneWidth =225;
  Mat imageoriginal;

  int endrow = 100;

  int pathtypecheck(TrackRecognition &track)
  {

    ipm_pathstype = pathstype::None;

    float heightoffset = 0.5;

    int i, j, s1, k1;

    //  for (j = 0; j < track.pointsEdgeLeft.size(); j++)
    //       // cout<<"s1=11111111111"<<endl;
    //   printf(" track.pointsEdgeLeft ---------%d  %d   %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);
    track.validRowsCal();
    //  for (j =track.validRowsRight; j < track.pointsEdgeRight.size(); j++)

    //   printf(" track.pointsEdgeLeft ---------%d  %d   %d\n",track.pointsEdgeRight[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);

    //////
    // track.pointsEdgeLeft.resize(j);
    //  track.pointsEdgeRight.resize(j);
    //////////////////-----------pathstype == pathstype:: pathRighttxie-----------------------------
    // printf("\n\n\n  -eeee-controlCenterCal--validRowsLeft = \n");
    printf("-0-concel-length--validRowsLeft=r--%d-- %d  ---%d\n", track.pointsEdgeLeft.size(), track.validRowsLeft, track.validRowsLeftend);

    printf("-0-concel--length---validRowsRight= r---%d  ---- %d  --%d\n", track.pointsEdgeRight.size(), track.validRowsRight, track.validRowsRightend);

    // if (track.pointsEdgeRight.size() > 50 && (track.pointsEdgeRight[track.validRowsRight].x - track.pointsEdgeLeft[track.validRowsLeft].x) > 120)
    // {
    //   s1 = 0;
    //   k1 = 0;
    //   for (j = track.validRowsRight; j < track.pointsEdgeRight.size() - 5; j++)
    //   {
    //     // cout<<"s1=11111111111"<<endl;
    //     // printf(" track.pointsEdgeLeft ---------%d  %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);
    //     if (track.pointsEdgeRight[j + 5].y < track.pointsEdgeRight[j].y)
    //     {
    //       s1 += 1;
    //       k1 += 1;
    //     }
    //     else
    //     {
    //       s1 = 0;
    //       break;
    //     }
    //   }

    //   if (s1 < 1 && j - track.validRowsRight > 10 || k1 > 50)
    //   {

    //     // track.pointsEdgeLeft.resize(j);
    //     // track.pointsEdgeRight.resize(j);

    //     endrow = j;
    //     //
    //     ipm_pathstype = pathstype::pathRightxie;
    //     printf("\n\n\n  ok-----------------pathstype::pathRightxie\n");
    //     printf("------Left.size()-----%d  %d---%d  %d\n", track.validRowsLeft, track.pointsEdgeLeft.size(), track.validRowsRight, track.pointsEdgeRight.size());

    //     return 0;
    //   }
    // }

    ////////////////////------------pathstype == pathstype:: pathRightxie-----------------
    // printf("\n\n\n  -dddd-controlCenterCal--validRowsLeft = \n");
    //////////////////-----------pathstype == pathstype:: pathLeftxie-----------------------------

    // printf("track.validRowsLeft<0.7*track.pointsEdgeLeft.size()---------%d  %d\n",track.validRowsLeft,track.pointsEdgeLeft.size());

    // if (track.pointsEdgeLeft.size() > 50 && (track.pointsEdgeLeft[track.validRowsLeft].x - track.pointsEdgeRight[track.validRowsRight].x) > 120)
    // {
    //   s1 = 0;
    //   for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 5; j++)
    //   {
    //     // cout<<"s1=11111111111"<<endl;
    //     // printf(" track.pointsEdgeLeft ---------%d  %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);
    //     if (track.pointsEdgeLeft[j + 5].y > track.pointsEdgeLeft[j].y)
    //     {
    //       s1 += 1;
    //     }
    //     else
    //     {
    //       s1 = 0;
    //       break;
    //     }
    //   }

    //   if (s1 < 1 && j - track.validRowsLeft > 10)
    //   {

    //     // track.pointsEdgeLeft.resize(j);
    //     // track.pointsEdgeRight.resize(j);

    //     ipm_pathstype = pathstype::pathLeftxie;
    //     printf("\n\n\n  ok-----------------pathstype::pathLeftedge\n");
    //     return 0;
    //   }
    // }

    ////////////////////------------pathstype == pathstype:: pathLeftxie-----------------

    track.validRowsCal();

    //   printf("-1-concel-length--validRowsLeft=r--%d-- %d  ---%d\n", track.pointsEdgeLeft.size(), track.validRowsLeft, track.validRowsLeftend);

    //   printf("-1-concel--length---validRowsRight= r---%d  ---- %d  --%d\n", track.pointsEdgeRight.size(), track.validRowsRight, track.validRowsRightend);

    //   //////
    // printf("\n\n\n  -cccc-controlCenterCal--validRowsLeft = \n");
    //   ////////////////////---------- pathstype = pathstype::pathStraight-----------------

    //
    if (track.validRowsLeft < 0.6 * track.pointsEdgeLeft.size() && track.validRowsRight < 0.6 * track.pointsEdgeRight.size() && sigma_valid(track.pointsEdgeLeft, track.validRowsLeft) < 1000 && sigma_valid(track.pointsEdgeRight, track.validRowsRight) < 1000 && vecmin(track.pointsEdgeLeft, track.validRowsLeft) > 0 && vecmax(track.pointsEdgeRight, track.validRowsRight) < 319)
    {

      //
      printf("\n\n-pathstype::pathStraighttrack.validRowsLeft-vecmax.x=%3.2f \t %3.2f\t  %d\n", sigma_valid(track.pointsEdgeLeft, track.validRowsLeft), sigma_valid(track.pointsEdgeRight, track.validRowsRight), vecmax(track.pointsEdgeRight, track.validRowsRight)); // 绿色点
      ipm_pathstype = pathstype::pathStraight;                                                                                                                                                                                                                          // 直入十字None;
      //
      return 0;
    }
    ////////////////////---------- pathstype = pathstype::pathStraight-----------------
    // printf("\n\n\n  -bbbb-controlCenterCal--validRowsLeft = \n");
    //////////////////-----------pathstype == pathstype::pathLeftedge-----------------------------

    // printf("track.validRowsLeft<0.7*track.pointsEdgeLeft.size()---------%d  %d\n",track.validRowsLeft,track.pointsEdgeLeft.size());

    // if ((track.validRowsLeft - track.validRowsRight < 120 || track.validRowsLeft < track.validRowsRight) && track.pointsEdgeLeft.size() > 50 && vecmin(track.pointsEdgeLeft, track.validRowsLeft) > 0 && track.validRowsLeft < 0.7 * track.pointsEdgeLeft.size())
    // {
    //   s1 = 0;
    //   for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 10; j = j + 10)
    //   {
    //     // cout<<"s1=11111111111"<<endl;
    //     // printf(" track.pointsEdgeLeft ---------%d  %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);
    //     if (track.pointsEdgeLeft[j + 10].y > track.pointsEdgeLeft[j].y || track.pointsEdgeLeft[track.validRowsLeft].y < track.pointsEdgeLeft[j + 5].y)
    //     {
    //       s1 += 1;
    //     }
    //     else
    //     {
    //       s1 = 0;
    //       break;
    //     }
    //   }

    //   if (s1 > 0)
    //   {
    //     ipm_pathstype = pathstype::pathLeftedge;
    //     printf("\n\n\n  ok-----------------pathstype::pathLeftedge\n");
    //     return 0;
    //   }
    // }

    ////////////////////------------pathstype == pathstype::pathLeftedge-----------------

    //  for (j = 0; j < track.pointsEdgeRight.size(); j++)
    //       // cout<<"s1=11111111111"<<endl;
    //   printf(" track.pointsEdgeLeft ---------%d  %d   %d\n",track.pointsEdgeRight[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);
    // printf("\n\n\n  -aaaa-controlCenterCal--validRowsLeft = \n");
    ////////////////////---------- pathstype = pathstype::pathRightedge----------------

    // if (track.validRowsLeft > track.validRowsRight && track.pointsEdgeRight.size() > 50 && track.validRowsRight < 0.8 * track.pointsEdgeRight.size() && vecmax(track.pointsEdgeRight, track.validRowsRight) < 319)
    // {
    //   s1 = 0;
    //   for (j = track.validRowsRight; j < track.pointsEdgeRight.size() - 12; j++)
    //   {
    //     // cout<<"s1=11111111111"<<endl;

    //     if (track.pointsEdgeRight[j + 10].y < track.pointsEdgeRight[j].y)
    //     {
    //       s1 += 1;
    //     }
    //     else
    //     {
    //       s1 = 0;
    //       break;
    //     }
    //   }
    //   if (s1 > 0)
    //   {
    //     ipm_pathstype = pathstype::pathRightedge; //
    //     printf("\n\n\n  -----------------pathstype ==pathstype::pathRightedge;\n");
    //     return 0;
    //   }
    // }

    ////////////////////---------- pathstype = pathstype::pathRightedge----------------

    // track.validRowsCal();
    // track = retrack(track);
    // printf("-1-concel---validRowsRight= r---%d   --%d\n",
    // track.validRowsRight, track.validRowsRightend);
    return 0;
    //////////////
  }

  void ipm_edge(TrackRecognition &trackRecognition, Mat &frame1) //
  {
    // frame0 = frame1;
    cv::Point2f point1, point2;
    int i = 1, j = 1;

    // cv::Mat image0= cv::Mat::zeros(800, 500, CV_8UC3);
    // if (frame0.empty())
    // {
    //   printf("Cannot open image! \n");
    //   sleep(20);
    // }

    /////////////////////////////////////////////--------------------------------

    /////////////////////////////////////////------------------------------------------

    Point2d startIpml, startIpmr;
    POINT startPoint;
    Point2d startIipm;
    centerEdge.clear();
    ipmpointsEdgeLeft.clear();
    ipmpointsEdgeRight.clear();
    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());

    for (int i = 0; i < trackRecognition.pointsEdgeRight.size(); i = i + 2)
    {
      startIpml = ipm.homography(Point2d(trackRecognition.pointsEdgeLeft[i].y, trackRecognition.pointsEdgeLeft[i].x));
      startIipm = ipm.homographyInv(startIpml); // 反透视变换
      startPoint = POINT(startIipm.y, startIipm.x);

      if (startPoint.y < 2)
        startIpml.x = 0;
      ipmpointsEdgeLeft.push_back(POINT(startIpml.y, startIpml.x));

      ///////////////////////////--------pointsEdgeRight

      startIpmr = ipm.homography(Point2d(trackRecognition.pointsEdgeRight[i].y, trackRecognition.pointsEdgeRight[i].x));
      startIipm = ipm.homographyInv(startIpmr); // 反透视变换
      startPoint = POINT(startIipm.y, startIipm.x);

      if (startPoint.y > 317)
        startIpmr.x = 319;
      ipmpointsEdgeRight.push_back(POINT(startIpmr.y, startIpmr.x));
    }
    //   for (int i =0; i <trackRecognition.pointsEdgeRight.size(); i++)
    //             {

    // printf("ipmpointsEdgei=%d- %d=\t%d=\t%d\t %d ---%d\n",i, ipmpointsEdgeLeft[i].x,ipmpointsEdgeLeft[i].y,  ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
    //             }

    // printf("\n\n -2-ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());
    ///////////////////--------------------------
  }




/////////////----------------------------------




TrackRecognition ipm_pathstype_world(TrackRecognition &trackRecognition, Mat &frame0, int pathstype) //
  {
    imageoriginal = frame0.clone();
  
    // printf("\n\n --11-  pathstype   =%d\n", pathstype);
    cv::Point2f point1, point2;

    int i = 1;
    float delta;
    int startrow = 0;
    int startrow2 = 0;

    if (frame0.empty())
    {
      cout << "Cannot open image!" << endl;
    }
    // --------------[02] 图像预处理
    // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
    // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    int w = 1000, h = 2000;

    cv::Size size(w * 0.2, h * 0.2); // 新尺寸为300x200
    cv::Mat image1, image0;          //

    ipmpointsEdgeLeft.clear();
    ipmpointsEdgeRight.clear();
    
       cv::Mat dstImg9 = cv::Mat::zeros(h, w, CV_8UC3);
    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    Scalar color(255, 255, 255); // 线条颜色（BGR格式）
    int thickness = 3;           // 线条宽度

    ////////////////////////////
   
      double fx = 256.389, fy = 256.3448;
      double cx = 162.6643, cy = 121.6213, xy = 0.1526; // 注意cx和cy与MATLAB代码中的行列对应关系
      double hpix = 336;

      double k11 = 0.9993, k12 = 0.0106, k13 = 0.036;
      double k21 = -0.0282, k22 = 0.8156, k23 = 0.5779;
      double k31 = -0.0241, k32 = -0.5685, k33 = 0.8223;

      // double a = deg2rad(33.69);
      //  处理左边的点
      if (pathstype == pathstype::pathLeftedge_only)
      {
        for (auto &point : trackRecognition.pointsEdgeLeft)
        {
          double u = point.y; // 假设u值存储在x成员中
          double v = point.x; // 假设v值存储在y成员中

          double Z = hpix / (k23 + k21 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k22 * (v - cy) / fy);

          double x = (k11 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k12 * (v - cy) / fy + k13) * Z;
          double z = (k31 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k32 * (v - cy) / fy + k33) * Z;

          POINT pointTmp(z, x);
          ipmpointsEdgeLeft.push_back(pointTmp);
          trackRecognition.ipmpointsEdgeLeft.push_back(pointTmp);
        }

        centerEdgereal = calCenterLine(ipmpointsEdgeLeft, pathstype);

        for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
        {
          circle(dstImg9, Point(ipmpointsEdgeLeft[i].y + w / 2, h - ipmpointsEdgeLeft[i].x),
                 5, colorLine1, 5); // 60行
        }

        for (int i = 0; i < centerEdgereal.size(); i++)
        {
          circle(dstImg9,
                 Point(centerEdgereal[i].y + w / 2, h - centerEdgereal[i].x), 15,
                 Scalar(0, 0, 255), 2); // 60行
          // printf("---Right i=%d- %d=\t%d=\t%d\t %d ---%d\n", i, ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y, ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
        }
      }
      else if (pathstype == pathstype::pathRightedge_only)
      {
        // 处理右边的点，与左边类似

        for (auto &point : trackRecognition.pointsEdgeRight)
        {

          double u = point.y; // 假设u值存储在x成员中
          double v = point.x; // 假设v值存储在y成员中

          double Z = hpix / (k23 + k21 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k22 * (v - cy) / fy);

          double x = (k11 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k12 * (v - cy) / fy + k13) * Z;
          double z = (k31 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k32 * (v - cy) / fy + k33) * Z;

          POINT pointTmp(z, x); // 缩放0.5
          ipmpointsEdgeRight.push_back(pointTmp);

          trackRecognition.ipmpointsEdgeRight.push_back(pointTmp);
        }

        centerEdgereal = calCenterLine(ipmpointsEdgeRight, pathstype);

        for (int i = 0; i < ipmpointsEdgeRight.size(); i++)
        {
          circle(dstImg9, Point(ipmpointsEdgeRight[i].y + w / 2, h - ipmpointsEdgeRight[i].x),
                 5, colorLine2, 5); // 60行
        }

        for (int i = 0; i < centerEdgereal.size(); i++)
        {
          circle(dstImg9,
                 Point(centerEdgereal[i].y + w / 2, h - centerEdgereal[i].x), 15,
                 Scalar(0, 0, 255), 2); // 60行
          // printf("---Right i=%d- %d=\t%d=\t%d\t %d ---%d\n", i, ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y, ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
        }
      }
      // 这里可以添加绘图或其他处理逻辑

     // printf("\n-- ipmpointsEdgeLeft.size()= %d \n", ipmpointsEdgeLeft.size());
      // for (i =0; i < pointsEdgeRightnew.size(); i++)
      // {
      //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
      // }
  

 


    ////////////

    // -------------------------------在图层上画线

    // 在图层上画线

    line(dstImg9, Point(0.5 * w, 0), Point(0.5 * w, h - 1), Scalar(255, 155, 255), thickness);             // 200
    line(dstImg9, Point(0.5 * w - 225, 0), Point(0.5 * w - 225, h - 1), Scalar(255, 155, 255), thickness); // 200

    line(dstImg9, Point(1, h * 0.1), Point(w - 1, h * 0.1), Scalar(255, 55, 255), 9);
    line(dstImg9, Point(1, h * 0.25), Point(w - 1, h * 0.25), color, 9); // h*0.8
    line(dstImg9, Point(1, h * 0.5), Point(w - 1, h * 0.5), color, thickness);
    line(dstImg9, Point(1, h * 0.75), Point(w - 1, h * 0.75), color, thickness);
    line(dstImg9, Point(1, h * 0.9), Point(w - 1, h * 0.9), color, 12); // h*0.8

    putText(dstImg9,
            formatInt2String(h * 0.9, 20) + "mm",
            Point(10, h * 0.1 - 20), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.75, 20) + "mm",
            Point(10, h * 0.25 - 20), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.5, 20) + "mm",
            Point(10, h * 0.5 - 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.25, 20) + "mm",
            Point(10, h * 0.75 - 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.1, 20) + "mm",
            Point(10, h * 0.9 + 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速

    // putText(dstImg9,
    //                      formatInt2String(1910, 3)+"cm" ,
    //                     Point(COLSIMAGE*0.5,12), FONT_HERSHEY_PLAIN, 1,
    //                     Scalar(100, 0, 255), 2); // 车速

    // imshow("ipm_world",dstImg9);
    /////////////////

    cv::resize(dstImg9, image0, size);

    imshow("ipm_control_world", image0);

    // printf("-1-concel---validRowsLeft=r--%d-----%d\n",
    //        trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
    // printf("-1-concel---validRowsRight= r---%d   --%d\n",
    //        trackRecognition.validRowsRight, trackRecognition.validRowsRightend);
    /*
      // 处理右边的点，与左边类似



    startrow=0;*/

    // if ( centerEdgereal.empty()) {
    //         std::cout << "The vector  centerEdgereal is empty." << std::endl;
    //     } else {

    //         std::cout << "The vector  centerEdgereal is not empty." << std::endl;
    //     }

    centerEdgereal.clear();

    POINT centermiao;

    int miaorow = 0, py = 0, pyreal = 0;
    startrow = 0;
    // startrow =MAX(trackRecognition.validRowsLeft,
    // trackRecognition.validRowsRight);
    // if (pathstype == pathstype::pathLeftxie)
    // {
    //   pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
    //                                trackRecognition.validRowsLeft,
    //                            pointsEdgeLeftnew.end());
    //   float lk = centerline_xielv(pointsEdgeLeftnew);
    //

    // 右斜入十字
    printf("\n\n\n\n  endrow---= %d   %d \n", endrow, trackRecognition.pointsEdgeRight.size());
    // for (int i = 0; i <endrow; i++)

    // for (int i = 0; i <trackRecognition.pointsEdgeRight.size(); i++)
    //  {
    //    if (pathstype == pathstype::pathStraight)
    //    {
    //      py = 0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) + w * 0.5;
    //      pyreal=0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y);
    //    }
    //    else if (pathstype == pathstype::pathLeftedge||pathstype == pathstype::pathLeftxie)
    //    {
    //      py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
    //      pyreal=pointsEdgeLeftnew[i].y +225;
    //    }

    //    else if (pathstype == pathstype::pathRightedge ||pathstype ==pathstype::pathRightxie)
    //    {

    //        py = pointsEdgeRightnew[i].y-225 + w * 0.5;
    //        pyreal=pointsEdgeRightnew[i].y -225;
    //    }



return trackRecognition;

  }

 
///////////-------------------------------------


TrackRecognition ipm_world(TrackRecognition &trackRecognition, Mat &frame0) //
  {
    imageoriginal = frame0.clone();
    // vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
    // vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

    // printf("\n\n --00-  pathstype   =%d\n", pathstype);

    // pathstype =pathstype::pathStraight // 直入十字None;
    pathtypecheck(trackRecognition);

    // printf("\n\n --11-  pathstype   =%d\n", pathstype);
    cv::Point2f point1, point2;

    int i = 1;
    float delta;
    int startrow = 0;
    int startrow2 = 0;

    if (frame0.empty())
    {
      cout << "Cannot open image!" << endl;
    }
    // --------------[02] 图像预处理
    // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
    // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    int w = 1000, h = 2000;

    cv::Size size(w * 0.2, h * 0.2); // 新尺寸为300x200
    cv::Mat image1, image0;          //

    ipmpointsEdgeLeft.clear();
    ipmpointsEdgeRight.clear();
    ////////////////////////////

    //////  image0=image1.clone();

    {
      double fx = 256.389, fy = 256.3448;
      double cx = 162.6643, cy = 121.6213, xy = 0.1526; // 注意cx和cy与MATLAB代码中的行列对应关系
      double hpix = 336;

      double k11 = 0.9993, k12 = 0.0106, k13 = 0.036;
      double k21 = -0.0282, k22 = 0.8156, k23 = 0.5779;
      double k31 = -0.0241, k32 = -0.5685, k33 = 0.8223;

      // double a = deg2rad(33.69);
      //  处理左边的点

      for (auto &point : trackRecognition.pointsEdgeLeft)
      {
        double u = point.y; // 假设u值存储在x成员中
        double v = point.x; // 假设v值存储在y成员中

        double Z = hpix / (k23 + k21 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k22 * (v - cy) / fy);

        double x = (k11 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k12 * (v - cy) / fy + k13) * Z;
        double z = (k31 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k32 * (v - cy) / fy + k33) * Z;

        POINT pointTmp(z, x);
        ipmpointsEdgeLeft.push_back(pointTmp);

trackRecognition.ipmpointsEdgeLeft.push_back(pointTmp);

      }

      // 处理右边的点，与左边类似
      for (auto &point : trackRecognition.pointsEdgeRight)
      {

        double u = point.y; // 假设u值存储在x成员中
        double v = point.x; // 假设v值存储在y成员中

        double Z = hpix / (k23 + k21 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k22 * (v - cy) / fy);

        double x = (k11 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k12 * (v - cy) / fy + k13) * Z;
        double z = (k31 * (u / fx - xy / (fx * fy) * v - (cx * fy - cy * xy) / (fx * fy)) + k32 * (v - cy) / fy + k33) * Z;

        POINT pointTmp(z, x); // 缩放0.5
        ipmpointsEdgeRight.push_back(pointTmp);

        trackRecognition.ipmpointsEdgeRight.push_back(pointTmp);
      }

      // 这里可以添加绘图或其他处理逻辑

      printf("\n-- ipmpointsEdgeLeft.size()= %d \n", ipmpointsEdgeLeft.size());
      // for (i =0; i < pointsEdgeRightnew.size(); i++)
      // {
      //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
      // }
    }

    cv::Mat dstImg9 = cv::Mat::zeros(h, w, CV_8UC3);
    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
    {
      circle(dstImg9, Point(ipmpointsEdgeLeft[i].y + w / 2, h - ipmpointsEdgeLeft[i].x),
             5, colorLine1, 5); // 60行

      circle(dstImg9,
             Point(ipmpointsEdgeRight[i].y + w / 2, h - ipmpointsEdgeRight[i].x), 5,
             colorLine2, 2); // 60行
      circle(dstImg9,
             Point((ipmpointsEdgeLeft[i].y + ipmpointsEdgeRight[i].y) * 0.5 + w / 2, h - ipmpointsEdgeRight[i].x), 8,
             Scalar(0, 0, 255), 2); // 60行
      printf("---Right i=%d- %d=\t%d=\t%d\t %d ---%d\n", i, ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y, ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
    }
    ////////////

    Scalar color(255, 255, 255); // 线条颜色（BGR格式）
    int thickness = 3;           // 线条宽度

    // -------------------------------在图层上画线

    // 在图层上画线

    line(dstImg9, Point(0.5 * w, 0), Point(0.5 * w, h - 1), Scalar(255, 155, 255), thickness);             // 200
    line(dstImg9, Point(0.5 * w - 225, 0), Point(0.5 * w - 225, h - 1), Scalar(255, 155, 255), thickness); // 200

    line(dstImg9, Point(1, h * 0.1), Point(w - 1, h * 0.1), Scalar(255, 55, 255), 9);
    line(dstImg9, Point(1, h * 0.25), Point(w - 1, h * 0.25), color, 9); // h*0.8
    line(dstImg9, Point(1, h * 0.5), Point(w - 1, h * 0.5), color, thickness);
    line(dstImg9, Point(1, h * 0.75), Point(w - 1, h * 0.75), color, thickness);
    line(dstImg9, Point(1, h * 0.9), Point(w - 1, h * 0.9), color, 12); // h*0.8

    putText(dstImg9,
            formatInt2String(h * 0.9, 20) + "mm",
            Point(10, h * 0.1 - 20), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.75, 20) + "mm",
            Point(10, h * 0.25 - 20), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.5, 20) + "mm",
            Point(10, h * 0.5 - 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.25, 20) + "mm",
            Point(10, h * 0.75 - 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9,
            formatInt2String(h * 0.1, 20) + "mm",
            Point(10, h * 0.9 + 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速

    // putText(dstImg9,
    //                      formatInt2String(1910, 3)+"cm" ,
    //                     Point(COLSIMAGE*0.5,12), FONT_HERSHEY_PLAIN, 1,
    //                     Scalar(100, 0, 255), 2); // 车速

    // imshow("ipm_world",dstImg9);
    /////////////////

    cv::resize(dstImg9, image0, size);

    imshow("ipm_world", image0);

    // printf("-1-concel---validRowsLeft=r--%d-----%d\n",
    //        trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
    // printf("-1-concel---validRowsRight= r---%d   --%d\n",
    //        trackRecognition.validRowsRight, trackRecognition.validRowsRightend);
    /*
      // 处理右边的点，与左边类似



    startrow=0;*/

    // if ( centerEdgereal.empty()) {
    //         std::cout << "The vector  centerEdgereal is empty." << std::endl;
    //     } else {

    //         std::cout << "The vector  centerEdgereal is not empty." << std::endl;
    //     }

    centerEdgereal.clear();

    POINT centermiao;

    int miaorow = 0, py = 0, pyreal = 0;
    startrow = 0;
    // startrow =MAX(trackRecognition.validRowsLeft,
    // trackRecognition.validRowsRight);
    // if (pathstype == pathstype::pathLeftxie)
    // {
    //   pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
    //                                trackRecognition.validRowsLeft,
    //                            pointsEdgeLeftnew.end());
    //   float lk = centerline_xielv(pointsEdgeLeftnew);
    //

    // 右斜入十字
    printf("\n\n\n\n  endrow---= %d   %d \n", endrow, trackRecognition.pointsEdgeRight.size());
    // for (int i = 0; i <endrow; i++)

    /*for (int i = 0; i <trackRecognition.pointsEdgeRight.size(); i++)
     {
       if (pathstype == pathstype::pathStraight)
       {
         py = 0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) + w * 0.5;
         pyreal=0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y);
       }
       else if (pathstype == pathstype::pathLeftedge||pathstype == pathstype::pathLeftxie)
       {
         py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
         pyreal=pointsEdgeLeftnew[i].y +225;
       }

       else if (pathstype == pathstype::pathRightedge ||pathstype ==pathstype::pathRightxie)
       {

           py = pointsEdgeRightnew[i].y-225 + w * 0.5;
           pyreal=pointsEdgeRightnew[i].y -225;
       }


   centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
   centerEdgereal.emplace_back(pointsEdgeRightnew[i].x,pyreal);


     }*/

    // for (int i = 0; i < centerEdge.size(); i++)
    //     {
    //      printf("centerEdge[i]i.x-y=   %d  %d   %d   %d \n",centerEdge[i].x,centerEdgereal[i].x,
    //      centerEdge[i].y,centerEdgereal[i].y);
    //     }

    // cv::resize(image0, image0, cv::Size((int)(w *0.25), (int)(h * 0.25)));
    // ///////////////////------------------
    // string imageName = "your_image_name.jpg";
    // MouseParams params;
    // params.img = image0;
    // params.imageName = imageName;
    // namedWindow("Image");
    // setMouseCallback("Image", onMouse, &params);

return trackRecognition;

  }


  void ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
  {
    // imageoriginal=frame0.clone();
    //  vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
    //  vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

    // printf("\n\n --00-  pathstype   =%d\n", pathstype);

    // pathstype =pathstype::pathStraight // 直入十字None;
    //
    pathtypecheck(trackRecognition);

    // printf("\n\n -pathtypecheck-  pathstype   =%d\n", pathstype);
    cv::Point2f point1, point2;

    int i = 1, j = 1;
    float delta;
    int startrow = 0;
    int startrow2 = 0;

    // cv::Mat image0= cv::Mat::zeros(800, 500, CV_8UC3);

    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    if (frame0.empty())
    {
      printf("Cannot open image! \n");
      sleep(20);
    }
    // --------------[02] 图像预处理
    // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
    // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    /////////////////////////////////////////////--------------------------------
    //
    cv::Mat dstImg9;
    // dstImg9 = cv::Mat::zeros(240, 320, CV_8UC3);
    // Mat dstImg9=frame0.clone();

    ipm.homographyInv(frame0, dstImg9, cv::BORDER_CONSTANT);

    /////////////////////////////////////////------------------------------------------
    ////////////////////////////////-------------------------------

    Point2d startIpml, startIpmr;
    //

    POINT startPoint;
    Point2d startIipm;

    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());

    ///////////////////--------------------------

    std::vector<cv::Point> centerLinePoints; // 存储计算出的中心线点
    int laneWidth = 35;                      // 假设车道宽度为50个像素单位
    cv::Point2f direction;

    for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
    {
      circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
             1, colorLine1, 5); // 60行

      circle(dstImg9,
             Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x), 1,
             colorLine2, 2); // 60行
    }

    if (ipmpointsEdgeLeft.size() > 50)
    {

      for (size_t i = 0; i < ipmpointsEdgeLeft.size() - 10; i = i + 5)
      {
        // 计算当前点到下一个点的向量
        direction = Point(ipmpointsEdgeLeft[i + 5].y - ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i + 5].x - ipmpointsEdgeLeft[i].x);

        // 单位化
        direction /= cv::norm(direction);

        // 计算垂直于车道线方向的单位向量（向右为正方向）
        cv::Point2f perpendicular(-direction.y, direction.x);

        // 对于每个点，根据车道宽度和垂直方向向量计算中心线上的点
        cv::Point centerPoint = Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x) + cv::Point(perpendicular.x * laneWidth, perpendicular.y * laneWidth);

        centerLinePoints.push_back(centerPoint);
      }

      // 为了简化处理，最后一个点采用与前一个相同的方向
      cv::Point lastPoint = Point(ipmpointsEdgeLeft.back().y, ipmpointsEdgeLeft.back().x) + cv::Point(
                                                                                                laneWidth * -direction.x, // 使用前一个计算好的方向
                                                                                                laneWidth * direction.y);
      centerLinePoints.push_back(lastPoint);

      // 创建一个空白图像

      // 绘制左边车道线

      // 绘制中心线
      for (size_t i = 0; i < centerLinePoints.size() - 1; i++)
      {
        cv::line(dstImg9, centerLinePoints[i], centerLinePoints[i + 1], cv::Scalar(0, 0, 255), 1); // 绿色线条
      }
    }

    /////////////////////////-----------------
    //////////////////////-----分割环岛内圆
    //--------------------------------- 读取偏移量
    // 创建窗口
    // cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    // // 设置鼠标回调函数
    // cv::setMouseCallback("Image", onMouse, reinterpret_cast<void*>(&dstImg9));

    // // 显示图片
    // cv::imshow("Image", dstImg9);

    //------------------------ 读取偏移量

    cv::imshow("guan fangku--Transformed", dstImg9);

    // cv::waitKey(0);
    ///////////////////////-----------------------------------
    //    for (int i =0; i <trackRecognition.pointsEdgeRight.size(); i++)
    //             {

    // printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, ipmpointsEdgeLeft[i].x,ipmpointsEdgeLeft[i].y,  ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
    //             }

    // ipmpointsEdgeLeft=pointsSortForX(ipmpointsEdgeLeft);

    //   for (int i =0; i <ipmpointsEdgeLeft.size(); i++)
    //             {

    // printf("Right i=%d- %d=\t%d\n",i, ipmpointsEdgeLeft[i].x,ipmpointsEdgeLeft[i].y);
    //             }

    // printf("-1-concel---validRowsLeft=r--%d-----%d\n",
    //        trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
    // printf("-1-concel---validRowsRight= r---%d   --%d\n",
    //        trackRecognition.validRowsRight, trackRecognition.validRowsRightend);
    /*
      // 处理右边的点，与左边类似



    startrow=0;*/
    // if ( centerEdgereal.empty()) {
    //         std::cout << "The vector  centerEdgereal is empty." << std::endl;
    //     } else {

    //         std::cout << "The vector  centerEdgereal is not empty." << std::endl;
    //     }

    //     centerEdgereal.clear();

    //   POINT centermiao;

    //   int miaorow = 0, py = 0,pyreal=0;
    //   startrow = 0;
    //   // startrow =MAX(trackRecognition.validRowsLeft,
    //   // trackRecognition.validRowsRight);
    //   // if (pathstype == pathstype::pathLeftxie)
    //   // {
    //   //   pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
    //   //                                trackRecognition.validRowsLeft,
    //   //                            pointsEdgeLeftnew.end());
    //   //   float lk = centerline_xielv(pointsEdgeLeftnew);
    //   //

    // // 右斜入十字
    // printf("\n\n\n\n  endrow---= %d   %d \n",endrow,trackRecognition.pointsEdgeRight.size());
    //   // for (int i = 0; i <endrow; i++)

    //  for (int i = 0; i <trackRecognition.pointsEdgeRight.size(); i++)
    //   {
    //     if (pathstype == pathstype::pathStraight)
    //     {
    //       py = 0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) + w * 0.5;
    //       pyreal=0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y);
    //     }
    //     else if (pathstype == pathstype::pathLeftedge||pathstype == pathstype::pathLeftxie)
    //     {
    //       py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
    //       pyreal=pointsEdgeLeftnew[i].y +225;
    //     }

    //     else if (pathstype == pathstype::pathRightedge ||pathstype ==pathstype::pathRightxie)
    //     {

    //         py = pointsEdgeRightnew[i].y-225 + w * 0.5;
    //         pyreal=pointsEdgeRightnew[i].y -225;
    //     }

    // centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
    // centerEdgereal.emplace_back(pointsEdgeRightnew[i].x,pyreal);

    //   }

    // for (int i = 0; i < centerEdge.size(); i++)
    //     {
    //      printf("centerEdge[i]i.x-y=   %d  %d   %d   %d \n",centerEdge[i].x,centerEdgereal[i].x,
    //      centerEdge[i].y,centerEdgereal[i].y);
    //     }

    // cv::resize(image0, image0, cv::Size((int)(w *0.25), (int)(h * 0.25)));
    // ///////////////////------------------
    // string imageName = "your_image_name.jpg";
    // MouseParams params;
    // params.img = image0;
    // params.imageName = imageName;
    // namedWindow("Image");
    // setMouseCallback("Image", onMouse, &params);
  }



  vector<POINT> calCenterLine(const vector<POINT> &points,int pathstype)
  {
 //
 
  printf("\n\n -calCenterLine pathstype   =%d\n", pathstype);
    vector<POINT> center_line;
    if (points.size() < 2)
      return center_line; // 需要至少两个点来计算斜率
    std::vector<double> slopes = calculateSlopes(points);

    // 打印斜率
    // for (double slope : slopes) {
    //     std::cout << "Slope: " << slope << std::endl;
    // }

    double averageSlope = std::accumulate(slopes.begin(), slopes.end(), 0.0) / slopes.size();
    double angle = atan(averageSlope);
 double  laneWidthcal=0; 
  if (pathstype==pathstype::pathLeftedge_only)//||   if (pathstype==pathstype::pathLeftedge_only)ipm_pathstype == pathstype::pathLeftxie 
    {
      laneWidthcal = -laneWidth;
    }
    else
     if (pathstype==pathstype::pathRightedge_only)
        laneWidthcal = laneWidth;
    // 计算偏移量
    double offsetX = -laneWidthcal * cos(angle + M_PI / 2);
    double offsetY = -laneWidthcal * sin(angle + M_PI / 2);

    // 应用偏移量计算中心线坐标

    center_line.clear();
    printf("-points=   %d\n", points.size());
    for (size_t i = 0; i < slopes.size() - 1; i++)
    { // 注意长度减一
      center_line.push_back(POINT(points[i].x + offsetX, points[i].y + offsetY));

      // printf("-center_line= %d---%d   --%d\n", i,center_line[i].x,center_line[i].y);
    }

    // 使用平均斜率和中心点计算截距
    return center_line;
  }

  void ipm_centeredge_cal(TrackRecognition &trackRecognition) //
  {
    // printf("\n\n -concal--pathtypecheck-  pathstype   =%d\n", pathstype);
    pathtypecheck(trackRecognition);

    int i = 1, j = 1;

    /////////////////////////////////////////------------------------------------------

    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());

    /////////////////-----------------------------------------------------------------------

    centerEdgereal.clear();
    int px, py;

    printf("\n\n\n  -8888888-controlCenterCal--validRowsLeft \n");

    for (int i = 0; i < ipmpointsEdgeRight.size(); i++)
    {
      if (ipm_pathstype == pathstype::pathStraight)
      {
        py = 0.5 * (ipmpointsEdgeLeft[i].y + ipmpointsEdgeRight[i].y);
        px = ipmpointsEdgeLeft[i].x;
      }
      else if (ipm_pathstype == pathstype::pathLeftedge_only)
      {
        py = ipmpointsEdgeLeft[i].y + 40;
        px = ipmpointsEdgeLeft[i].x;
      }

      else if (ipm_pathstype == pathstype::pathRightedge_only)
      {

        py = ipmpointsEdgeRight[i].y - 40;
        px = ipmpointsEdgeRight[i].x;
      }
      centerEdgereal.emplace_back(px, py);
    }

    /////////--------------保存使用，检测用，常规跑图要注释掉！！！！

    // savePointsToFile(centerEdgereal, "centerEdgereal.txt");

    /////////--------------保存使用，检测用，常规跑图要注释掉！！！！

    ///////////////-------------------------------------------------------------------------

    POINT startPoint;
    Point2d startIipm;

    //  printf("\n\n ---- centerEdgeipm.size()-11-====%d=\n\n\n" , centerEdgeipm2.size() );

    //    //
    centerEdge.clear();
    // centerEdge.reserve(300);
    //   printf("\n\n ---- centerEdgeipm.size()-22-====%d==\n\n\n" , centerEdgeipm2.size() );
    printf("\n\n\n  -99999999-controlCenterCal--validRowsLeft \n");

    if (centerEdgereal.size() > 50)
    {

      for (int i = 0; i < centerEdgereal.size(); i = i + 1)
      {

        startIipm = ipm.homographyInv(Point2d(centerEdgereal[i].y, centerEdgereal[i].x)); // 反透视变换
        startPoint = POINT(startIipm.y, startIipm.x);

        // if (startPoint.y < 319 && startPoint.y > 0 && startPoint.x > 0 && startPoint.x < 240)

        // centerEdgeipm2.push_back(startPoint);
        centerEdge.push_back(startPoint);

        // centerEdge.insert(centerEdge.begin()+i, startPoint);
        ///////////////////////////--------pointsEdgeRight
      }
    }
  }

  int ipm_check_ring(Mat &frame0) //
  {

    // printf("\n\n --11-  pathstype   =%d\n", pathstype);
    cv::Point2f point1, point2;

    int i = 1, j = 1;
    int8_t ringyes = 0;
    int startrow = 0;
    int startrow2 = 0;

    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    // if (frame0.empty())
    // {
    //   printf("Cannot open image! \n");
    //   sleep(20);
    // }
    // --------------[02] 图像预处理

    /////////////////////////////////////////////--------------------------------
    //
    cv::Mat dstImg9 = cv::Mat::zeros(240, 320, CV_8UC3);
    ;

    // ipm.homographyInv(frame0, dstImg9, cv::BORDER_CONSTANT);

    /////////////////////////////////////////------------------------------------------
    ////////////////////////////////-------------------------------

    Point2d startIpml, startIpmr;
    //

    POINT startPoint;
    Point2d startIipm;

    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());

    // for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
    // {

    //   printf("ipmpointsEdgei=%d- %d=\t%d=\t%d\t %d ---%d\n", i, ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y, ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
    // }

    // printf("\n\n -2-ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());

    ///////////////////--------------------------

    /////////////////////////-----------------
    //////////////////////-----分割环岛内圆
    int logo = 0, s1 = 0, s2 = 0, cnt = 0;
    for (i = 0; i < ipmpointsEdgeRight.size(); i++)
    {

      if (ipmpointsEdgeRight[i].y > 318)
        cnt += 1;
      else
        cnt = 0;

      if (cnt > 20)
        break;
    }
    if (cnt > 20)
    {
      cnt = 0;
      for (j = i; j < ipmpointsEdgeRight.size(); j++)
      {

        if (ipmpointsEdgeRight[j].y < 280)
        {
          s1 = j;
          break;
        }
      }
      cnt = 0;
      for (j = s1; j < ipmpointsEdgeRight.size(); j++)
      {

        if (ipmpointsEdgeRight[j].y < 270)
          cnt += 1;
        else
          cnt = 0;

        if (cnt > 20)
          break;
      }
    }

    if (cnt == 21)
    {

      for (i = j; i < ipmpointsEdgeRight.size(); i++)
      {
        if (ipmpointsEdgeRight[i].y > 317)
        {
          s2 = i - 1;
          break;
        }
        else if (ipmpointsEdgeRight[i].y < 317 && i == ipmpointsEdgeRight.size() - 1)
        {
          s2 = i;
        }
      }

      if (s2 > 0)
      {
        logo = 1;
      }
    }
    ///////////////////
    printf("--------------Right s1=%d\t s2=%d \n", s1, s2);
    if (logo > 0 && s1 > 0 && s2 - s1 > 20)
    {

      printf("Right s1=%d\t s2=%d \n", s1, s2);

      for (i = s1; i < s2; i++)
      {
        circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
               5, colorLine1, 5); // 60行

        circle(dstImg9,
               Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x), 5,
               colorLine2, 2); // 60行
                               // printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, ipmpointsEdgeLeft[i].x,ipmpointsEdgeLeft[i].y,  ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
      }

      ///////////////////
      //////////////////////-----分割环岛内圆
      /////////////////////////---------------------------------------------

      // 生成一些二维离散点作为示例数据
      // Mat points(s2-s1+1, 1, CV_32FC2);
      std::vector<cv::Point> points;
      for (int i = s1; i < s2; i++)
      {
        int x = static_cast<int>(ipmpointsEdgeRight[i].x);
        int y = static_cast<int>(ipmpointsEdgeRight[i].y);
        points.push_back(cv::Point(y, x));
      }
      // 生成点集

      // 检查点集是否至少包含5个点
      if (points.size() < 5)
      {
        std::cerr << "Need at least 5 points for fitEllipse" << std::endl;
        // return 0;
      }

      // 使用fitEllipse拟合点集
      cv::RotatedRect ellipse = cv::fitEllipse(points);

      cv::ellipse(dstImg9, ellipse, Scalar(0, 255, 0), 2);

      // imshow("Fitted Arc", img);

      // 计算拟合误差
      double error = 0.0;
      cv::Point2f ellipseCenter = ellipse.center;
      float avgRadius = (ellipse.size.width + ellipse.size.height) / 4; // 长轴和短轴的平均值除以2
      for (const auto &point : points)
      {
        double distance = cv::norm(cv::Point2f(point.x, point.y) - ellipseCenter);
        error += std::pow(distance - avgRadius, 2);
      }

      printf("Fitting error: %f\n", error);
      printf("Ellipse center: (%f, %f)\n", ellipse.center.x, ellipse.center.y);
      printf("Ellipse average radius: %f\n", avgRadius);
      printf("Ellipse average radius width-height: %3.2f ::\t  %3.2f \n", ellipse.size.width * 0.5, ellipse.size.height * 0.5);

      if (avgRadius > 30 && avgRadius < 80 && error < 8000 && abs(ellipse.size.width - ellipse.size.height) < 30)

        ringyes = 1;

      ////////////////////////-------------------------------------
    }
    for (int i = 0; i < ipmpointsEdgeLeft.size(); i = i + 2)
    {
      circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
             1, colorLine1, 5); // 60行

      circle(dstImg9,
             Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x), 1,
             colorLine2, 2); // 60行

      circle(dstImg9,
             Point(ipmpointsEdgeRight[i].y - 35, ipmpointsEdgeRight[i].x), 1,
             Scalar(0, 0, 255), 2);
      //  {
      //     circle(dstImg9,Point(trackRecognition.pointsEdgeLeft[i].y,trackRecognition.pointsEdgeLeft[i].x),8, colorLine1, 1); // 60行

      //     circle(dstImg9,Point(trackRecognition.pointsEdgeRight[i].y, trackRecognition.pointsEdgeRight[i].x),8, colorLine2, 1); // 60行

      //   }
    }

    //--------------------------------- 读取偏移量
    // 创建窗口
    // cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    // // 设置鼠标回调函数
    // cv::setMouseCallback("Image", onMouse, reinterpret_cast<void*>(&dstImg9));

    // // 显示图片
    // cv::imshow("Image", dstImg9);

    //------------------------ 读取偏移量
    /**/
    cv::imshow("guan fangku--Transformed", dstImg9);

    return ringyes;
  }

  void ipm_huitu(TrackRecognition &trackRecognition) //
  {

    cv::Point2f point1, point2;

    int i = 1, j = 1;
    float delta;
    int startrow = 0;
    int startrow2 = 0;

    // cv::Mat image0= cv::Mat::zeros(800, 500, CV_8UC3);

    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    // if (frame0.empty())
    // {
    //   printf("Cannot open image! \n");
    //   sleep(20);
    // }
    // --------------[02] 图像预处理
    // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
    // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    /////////////////////////////////////////////--------------------------------
    //
    cv::Mat dstImg9;
    // dstImg9 = cv::Mat::zeros(240, 320, CV_8UC3);
    // Mat dstImg9=frame0.clone();

    ipm.homographyInv(frame0, dstImg9, cv::BORDER_CONSTANT);

    /////////////////////////////////////////------------------------------------------
    ////////////////////////////////-------------------------------

    Point2d startIpml, startIpmr;
    //

    POINT startPoint;
    Point2d startIipm;

    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n", trackRecognition.pointsEdgeRight.size());

    ///////////////////--------------------------

    centerEdgereal.clear();
    printf("\n\n\n  -0-centerEdgereal.size()  =%d \n", centerEdgereal.size());
    // if (ipm_pathstype == pathstype::pathRightedge_only )//|| ipm_pathstype == pathstype::pathRightxie
    // {
    //   centerEdgereal = calCenterLine(ipmpointsEdgeRight);
    // }

    // else if (ipm_pathstype == pathstype::pathLeftedge_only )//|| ipm_pathstype == pathstype::pathLeftxie
    // {
    //   centerEdgereal = calCenterLine(ipmpointsEdgeLeft);
    // }

    // else
    // {
    //   centerEdgereal = calCenterLine(ipmpointsEdgeRight);
    // }

    printf("\n\n\n  -1-centerEdgereal.size()  =%d \n", centerEdgereal.size());
    // for (int i =0; i <ipmpointsEdgeRight.size(); i++)
    //             {

    // printf("ipmpointsEdgeRight=%d     =%d\t =%d\n",i,ipmpointsEdgeRight[i].x,ipmpointsEdgeRight[i].y);
    //             }

    //  for (int i =0; i <centerEdgereal.size(); i++)
    //             {

    // printf("centerEdgereali=%d     =%d\t =%d\n",i,centerEdgereal[i].x,centerEdgereal[i].y);
    //             }

    // 假设车道宽度为50个像素单位

    for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
    {
      circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
             1, colorLine1, 5); // 60行

      circle(dstImg9,
             Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x), 1,
             colorLine2, 2); // 60行
    }

    for (int i = 0; i < centerEdgereal.size(); i++)
    {
      circle(dstImg9, Point(centerEdgereal[i].y, centerEdgereal[i].x), 8, Scalar(255, 0, 0), 1);
    }

    /////////////////////////-----------------
    //////////////////////-----分割环岛内圆
    //--------------------------------- 读取偏移量
    // 创建窗口
    // cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    // // 设置鼠标回调函数
    // cv::setMouseCallback("Image", onMouse, reinterpret_cast<void*>(&dstImg9));

    // // 显示图片
    // cv::imshow("Image", dstImg9);

    //------------------------ 读取偏移量

    cv::imshow("guan fangku--Transformed", dstImg9);

    // cv::waitKey(0);
    ///////////////////////-----------------------------------
  }
};