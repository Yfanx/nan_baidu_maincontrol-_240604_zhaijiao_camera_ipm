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

class Edgeipm {
private:
  int counterShift = 0; // 变速计数器
public:
  vector<POINT> ipmpointsEdgeLeft, ipmpointsEdgeRight;

  Mat frame0;
 int ipm_controlCenter;
  pathstype ipm_pathstype = pathstype::None;
  vector<POINT> pointsEdgeLeftnew;  // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew; // 赛道右边缘点集
  vector<POINT> centerEdge, centerEdgereal;
  int laneWidth = 225;
  Mat imageoriginal;

  int endrow = 100;

  int pathtypecheck(TrackRecognition &track) {

    // ipm_pathstype = pathstype::None;

    float heightoffset = 0.5;
    int i, j, s1, k1;

    //  for (j = 0; j < track.pointsEdgeLeft.size(); j++)
    //       // cout<<"s1=11111111111"<<endl;
    //   printf(" track.pointsEdgeLeft ---------%d  %d
    //   %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);
    track.validRowsCal();
    //  for (j =track.validRowsRight; j < track.pointsEdgeRight.size(); j++)
    //   printf(" track.pointsEdgeLeft ---------%d  %d
    //   %d\n",track.pointsEdgeRight[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);


    printf("-0-concel-length--validRowsLeft=r--%d-- %d  ---%d\n",
           track.pointsEdgeLeft.size(), track.validRowsLeft,
           track.validRowsLeftend);

    printf("-0-concel--length---validRowsRight= r---%d  ---- %d  --%d\n",
           track.pointsEdgeRight.size(), track.validRowsRight,
           track.validRowsRightend);

    if (track.pointsEdgeLeft.size() < 4 || track.pointsEdgeRight.size() < 4) {

      printf("\n\n-pathstype::pathStraighttrack.validRowsLeft-vecmax.x=%3.2f "
             "\t %3.2f\t  %d\n",
             sigma_valid(track.pointsEdgeLeft, track.validRowsLeft),
             sigma_valid(track.pointsEdgeRight, track.validRowsRight),
             vecmax(track.pointsEdgeRight, track.validRowsRight)); // 绿色点
      ipm_pathstype = pathstype::pathStraight; // 直入十字None;
      //
      return 0;
    }

 
    ////////////////////------------pathstype == pathstype::
    ///pathRightxie-----------------
 
    if (track.validRowsLeft < 0.6 * track.pointsEdgeLeft.size() &&
        track.validRowsRight < 0.6 * track.pointsEdgeRight.size() &&
        sigma_valid(track.pointsEdgeLeft, track.validRowsLeft) < 1000 &&
        sigma_valid(track.pointsEdgeRight, track.validRowsRight) < 1000 &&
        vecmin(track.pointsEdgeLeft, track.validRowsLeft) > 0 &&
        vecmax(track.pointsEdgeRight, track.validRowsRight) < 319) {

      printf("\n\n-pathstype::pathStraighttrack.validRowsLeft-vecmax.x=%3.2f "
             "\t %3.2f\t  %d\n",
             sigma_valid(track.pointsEdgeLeft, track.validRowsLeft),
             sigma_valid(track.pointsEdgeRight, track.validRowsRight),
             vecmax(track.pointsEdgeRight, track.validRowsRight)); // 绿色点
      ipm_pathstype = pathstype::pathStraight; // 直入十字None;
      return 0;
    }
  
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
    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n",
    // trackRecognition.pointsEdgeRight.size());

    for (int i = 0; i < trackRecognition.pointsEdgeRight.size(); i = i + 2) {
      startIpml = ipm.homography(Point2d(trackRecognition.pointsEdgeLeft[i].y,
                                         trackRecognition.pointsEdgeLeft[i].x));
      startIipm = ipm.homographyInv(startIpml); // 反透视变换
      startPoint = POINT(startIipm.y, startIipm.x);

      if (startPoint.y < 2)
        startIpml.x = 0;
      ipmpointsEdgeLeft.push_back(POINT(startIpml.y, startIpml.x));

      ///////////////////////////--------pointsEdgeRight

      startIpmr =
          ipm.homography(Point2d(trackRecognition.pointsEdgeRight[i].y,
                                 trackRecognition.pointsEdgeRight[i].x));
      startIipm = ipm.homographyInv(startIpmr); // 反透视变换
      startPoint = POINT(startIipm.y, startIipm.x);

      if (startPoint.y > 317)
        startIpmr.x = 319;
      ipmpointsEdgeRight.push_back(POINT(startIpmr.y, startIpmr.x));
    }
    //   for (int i =0; i <trackRecognition.pointsEdgeRight.size(); i++)
    //             {

    // printf("\n\n -2-ipm-trackRecognition.pointsEdgeRight.size()   =%d\n",
    // trackRecognition.pointsEdgeRight.size());
    ///////////////////--------------------------
  }

  /////////////----------------------------------


  TrackRecognition ipm_pathstype_world(TrackRecognition &trackRecognition,
                                       Mat &frame0, int pathstype) //
  {
    imageoriginal = frame0.clone();
    printf("\n\n -ipm_pathstype_world   =%d\n", pathstype);
    cv::Point2f point1, point2;

    int i = 1;
    float delta;
    int startrow = 0;
    int startrow2 = 0;

    if (frame0.empty()) {
      cout << "Cannot open image!" <<endl;
    }
    // --------------[02] 图像预处理

    // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    int w = 1000, h = 2000;
    cv::Size size(w*0.2, h*0.2); // 新尺寸为300x200
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

    // double fx = 269.2319, fy = 268.8793;
    // double cx = 165.7599, cy = 113.3684,
    //        xy = -0.0088; // 注意cx和cy与MATLAB代码中的行列对应关系
    // double hpix = 332;

    double fx = 271.2319, fy = 271.2416;
    double cx = 166.5652, cy = 113.1121,
           xy = -0.0257; // 注意cx和cy与MATLAB代码中的行列对应关系
    double hpix = 332;


    // double k11 = 0.9996, k12 = -0.0144, k13 = 0.0225;
    // double k21 = -0.0002, k22 = 0.8382, k23 = 0.5453;
    // double k31 = -0.0267, k32 = -0.5451, k33 = 0.8379;
    double k11 = 1.0000, k12 = 0.0056, k13 = -0.0048;
    double k21 = -0.0021, k22 = 0.8406, k23 = 0.5416;
    double k31 = 0.0071, k32 = -0.5416, k33 = 0.8406;





    //////////

 trackRecognition.validRowsCal();
    printf("66-Left--Right.size()= %d   =%d  \n \n",
        trackRecognition.pointsEdgeLeft.size(), trackRecognition.pointsEdgeRight.size());

printf("-1-concel---validRowsLeft=r--%d-----%d\n", trackRecognition.validRowsLeft,trackRecognition.validRowsRight);
printf("-1-concel---validRowsLeftend --%d-----%d\n", trackRecognition.validRowsLeftend,trackRecognition.validRowsRightend);
    //
printf("\n\n ccc-ipm_pathstype_world   =%d\n", pathstype);
if (trackRecognition.validRowsLeftend> trackRecognition.validRowsLeft)
{
 trackRecognition.pointsEdgeLeft.erase(
        trackRecognition.pointsEdgeLeft.begin() +
            trackRecognition.validRowsLeftend,
        trackRecognition.pointsEdgeLeft.end()); // 删除前50行后的数据
    trackRecognition.pointsEdgeLeft.erase(
        trackRecognition.pointsEdgeLeft.begin(),
        trackRecognition.pointsEdgeLeft.begin() +
            trackRecognition.validRowsLeft);
}
   
printf("\n\n dddd-ipm_pathstype_world   =%d\n", pathstype);

if (trackRecognition.validRowsRightend>trackRecognition.validRowsRight)
{
      trackRecognition.pointsEdgeRight.erase(
        trackRecognition.pointsEdgeRight.begin() +
            trackRecognition.validRowsRightend,
        trackRecognition.pointsEdgeRight.end()); // 删除前50行后的数据
    trackRecognition.pointsEdgeRight.erase(
        trackRecognition.pointsEdgeRight.begin(),
        trackRecognition.pointsEdgeRight.begin() +
            trackRecognition.validRowsRight);
}
    printf("--bbbb--validRowsLeft =%d \t %d \t  %d\n", trackRecognition.validRowsLeft,
           trackRecognition.pointsEdgeLeft.size(), trackRecognition.pointsEdgeRight.size());
    ///////////

    // double a = deg2rad(33.69);
    //  处理左边的点
    if (pathstype == pathstype::pathLeftedge_only) {
      for (auto &point : trackRecognition.pointsEdgeLeft) {
        double u = point.y; // 假设u值存储在x成员中
        double v = point.x; // 假设v值存储在y成员中

        double Z = hpix / (k23 +
                           k21 * (u / fx - xy / (fx * fy) * v -
                                  (cx * fy - cy * xy) / (fx * fy)) +
                           k22 * (v - cy) / fy);

        double x = (k11 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k12 * (v - cy) / fy + k13) *
                   Z;
        double z = (k31 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k32 * (v - cy) / fy + k33) *
                   Z;

        POINT pointTmp(z, x);
        ipmpointsEdgeLeft.push_back(pointTmp);
        trackRecognition.ipmpointsEdgeLeft.push_back(pointTmp);
      }

      centerEdgereal =
          calCenterLine(ipmpointsEdgeLeft, ipmpointsEdgeLeft, pathstype);

      for (int i = 0; i < ipmpointsEdgeLeft.size(); i++) {
        circle(
            dstImg9,
            Point(ipmpointsEdgeLeft[i].y + w / 2, h - ipmpointsEdgeLeft[i].x),
            5, colorLine1, 5); // 60行
      }

      for (int i = 0; i < centerEdgereal.size(); i++) {
        circle(dstImg9,
               Point(centerEdgereal[i].y + w / 2, h - centerEdgereal[i].x), 15,
               Scalar(0, 0, 255), 2); // 60行
        // printf("---Right i=%d- %d=\t%d=\t%d\t %d ---%d\n", i,
        // ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y,
        // ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,
        // ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
      }
    } else if (pathstype == pathstype::pathRightedge_only) {
      // 处理右边的点，与左边类似

      for (auto &point : trackRecognition.pointsEdgeRight) {

        double u = point.y; // 假设u值存储在x成员中
        double v = point.x; // 假设v值存储在y成员中

        double Z = hpix / (k23 +
                           k21 * (u / fx - xy / (fx * fy) * v -
                                  (cx * fy - cy * xy) / (fx * fy)) +
                           k22 * (v - cy) / fy);

        double x = (k11 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k12 * (v - cy) / fy + k13) *
                   Z;
        double z = (k31 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k32 * (v - cy) / fy + k33) *
                   Z;

        POINT pointTmp(z, x); // 缩放0.5
        ipmpointsEdgeRight.push_back(pointTmp);

        trackRecognition.ipmpointsEdgeRight.push_back(pointTmp);
      }
      centerEdgereal =
          calCenterLine(ipmpointsEdgeRight, ipmpointsEdgeRight, pathstype);


      for (int i = 0; i < ipmpointsEdgeRight.size(); i++) {
        circle(
            dstImg9,
            Point(ipmpointsEdgeRight[i].y + w / 2, h - ipmpointsEdgeRight[i].x),
            5, colorLine2, 5); // 60行
      }


 for (int i = 0; i < ipmpointsEdgeLeft.size(); i++) {
        circle(
            dstImg9,
            Point(ipmpointsEdgeLeft[i].y + w / 2, h - ipmpointsEdgeLeft[i].x),
            2, colorLine1, 5); // 60行
      }

   for (int i = 0; i < ipmpointsEdgeRight.size(); i++) {
        circle(
            dstImg9,
            Point(ipmpointsEdgeRight[i].y + w / 2, h - ipmpointsEdgeRight[i].x),
            2, colorLine2, 3); // 60行
      }

   for (int i = 0; i < centerEdgereal.size(); i++) {
        circle(dstImg9,
               Point(centerEdgereal[i].y + w / 2, h - centerEdgereal[i].x), 15,
               Scalar(0, 0, 255), 2); // 60行


    } 
    }else /////-----------2edge
    {
      for (auto &point : trackRecognition.pointsEdgeLeft) {
        double u = point.y; // 假设u值存储在x成员中
        double v = point.x; // 假设v值存储在y成员中

        double Z = hpix / (k23 +
                           k21 * (u / fx - xy / (fx * fy) * v -
                                  (cx * fy - cy * xy) / (fx * fy)) +
                           k22 * (v - cy) / fy);

        double x = (k11 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k12 * (v - cy) / fy + k13) *
                   Z;
        double z = (k31 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k32 * (v - cy) / fy + k33) *
                   Z;

        POINT pointTmp(z, x);
        ipmpointsEdgeLeft.push_back(pointTmp);
        trackRecognition.ipmpointsEdgeLeft.push_back(pointTmp);
      }

      for (auto &point : trackRecognition.pointsEdgeRight) {

        double u = point.y; // 假设u值存储在x成员中
        double v = point.x; // 假设v值存储在y成员中

        double Z = hpix / (k23 +
                           k21 * (u / fx - xy / (fx * fy) * v -
                                  (cx * fy - cy * xy) / (fx * fy)) +
                           k22 * (v - cy) / fy);

        double x = (k11 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k12 * (v - cy) / fy + k13) *
                   Z;
        double z = (k31 * (u / fx - xy / (fx * fy) * v -
                           (cx * fy - cy * xy) / (fx * fy)) +
                    k32 * (v - cy) / fy + k33) *
                   Z;

        POINT pointTmp(z, x); // 缩放0.5
        ipmpointsEdgeRight.push_back(pointTmp);

        trackRecognition.ipmpointsEdgeRight.push_back(pointTmp);
      }

      centerEdgereal =
          calCenterLine(ipmpointsEdgeLeft, ipmpointsEdgeRight, pathstype);
      for (int i = 0; i < ipmpointsEdgeLeft.size(); i++) {
        circle(
            dstImg9,
            Point(ipmpointsEdgeLeft[i].y + w / 2, h - ipmpointsEdgeLeft[i].x),
            5, colorLine1, 5); // 60行
      }
      for (int i = 0; i < ipmpointsEdgeRight.size(); i++) {
        circle(
            dstImg9,
            Point(ipmpointsEdgeRight[i].y + w / 2, h - ipmpointsEdgeRight[i].x),
            5, colorLine2, 5); // 60行
      }
      for (int i = 0; i < centerEdgereal.size(); i++) {
        circle(dstImg9,
               Point(centerEdgereal[i].y + w / 2, h - centerEdgereal[i].x), 15,
               Scalar(0, 0, 255), 2); // 60行
        // printf("---Right i=%d- %d=\t%d=\t%d\t %d ---%d\n", i,
        // ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y,
        // ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,
        // ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
      }

      //////////////////////----2edge
    }
    // 这里可以添加绘图或其他处理逻辑
savePointsToFile(centerEdgereal, "centerEdgereal.txt");
    // printf("\n-- ipmpointsEdgeLeft.size()= %d \n", ipmpointsEdgeLeft.size());
    // for (i =0; i < pointsEdgeRightnew.size(); i++)
    // {
    //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i,
    //  trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x,
    //  pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y
    //  - pointsEdgeLeftnew[i].y);
    // }

    putText(dstImg9, formatInt2String((int)(ipm_controlCenter), 20) + "mm",
            Point(50, h * 0.5 + 190), FONT_HERSHEY_PLAIN, 10, Scalar(0, 255, 0),
            10); // 车速
    ////////////


    
        string str="pst: " + formatDoble2String(pathstype, 0) ;
        putText(dstImg9, str, Point(100,h * 0.5 - 180), FONT_HERSHEY_PLAIN, 10, Scalar(0, 255, 0), 10); // 赛道类型
        //      std:string str0 = std::to_string(servoPwm);

      

    // -------------------------------在图层上画线

    // 在图层上画线

    line(dstImg9, Point(0.5 * w, 0), Point(0.5 * w, h - 1),
         Scalar(255, 155, 255), thickness); // 200
    line(dstImg9, Point(0.5 * w - 225, 0), Point(0.5 * w - 225, h - 1),
         Scalar(255, 155, 255), thickness); // 200

    line(dstImg9, Point(1, h * 0.1), Point(w - 1, h * 0.1),
         Scalar(255, 55, 255), 9);
    line(dstImg9, Point(1, h * 0.25), Point(w - 1, h * 0.25), color,
         9); // h*0.8
    line(dstImg9, Point(1, h * 0.5), Point(w - 1, h * 0.5), color, thickness);
    line(dstImg9, Point(1, h * 0.75), Point(w - 1, h * 0.75), color, thickness);
    line(dstImg9, Point(1, h * 0.9), Point(w - 1, h * 0.9), color, 12); // h*0.8

    putText(dstImg9, formatInt2String(h * 0.9, 20) + "mm",
            Point(10, h * 0.1 - 20), FONT_HERSHEY_PLAIN, 5, Scalar(100, 0, 255),
            5); // 车速
    putText(dstImg9, formatInt2String(h * 0.75, 20) + "mm",
            Point(10, h * 0.25 - 20), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9, formatInt2String(h * 0.5, 20) + "mm",
            Point(10, h * 0.5 - 10), FONT_HERSHEY_PLAIN, 5, Scalar(100, 0, 255),
            5); // 车速
    putText(dstImg9, formatInt2String(h * 0.25, 20) + "mm",
            Point(10, h * 0.75 - 10), FONT_HERSHEY_PLAIN, 5,
            Scalar(100, 0, 255), 5); // 车速
    putText(dstImg9, formatInt2String(h * 0.1, 20) + "mm",
            Point(10, h * 0.9 + 10), FONT_HERSHEY_PLAIN, 5, Scalar(100, 0, 255),
            5); // 车速
    /////////////////

    cv::resize(dstImg9, image0, size);

    imshow("ipm_control_world", image0);

    printf("-ipm---validRowsLeft=r--%d-----%d\n",
           trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);

    centerEdgereal.clear();
    
    return trackRecognition;
  }

  ///////////-------------------------------------

  //   startrow=0;

  

 
  vector<POINT> calCenterLine(const vector<POINT> &points,
                              const vector<POINT> &points2, int pathstype) {
    //

    printf("\n\n -calCenterLine pathstype   =%d\n", pathstype);
    
    vector<POINT> center_line;
    
    if (points.size() < 10&&points2.size()<10)
      return center_line; // 需要至少两个点来计算斜率




    std::vector<double> slopes = calculateSlopes(points);

    // 打印斜率
    // for (double slope : slopes) {
    //     std::cout << "Slope: " << slope << std::endl;
    // }

    double averageSlope =
        std::accumulate(slopes.begin(), slopes.end(), 0.0) / slopes.size();
    double angle = atan(averageSlope);
    double laneWidthcal = 0;
    if (pathstype == pathstype::pathLeftedge_only)
    {
      laneWidthcal = -laneWidth;
    } else if (pathstype == pathstype::pathRightedge_only)
      laneWidthcal = laneWidth;
    // 计算偏移量

    if (pathstype == pathstype::pathLeftedge_only ||pathstype == pathstype::pathRightedge_only) {

      double offsetX = -laneWidthcal * cos(angle + M_PI / 2);
      double offsetY = -laneWidthcal * sin(angle + M_PI / 2);

      // 应用偏移量计算中心线坐标

      center_line.clear();
      printf("-points=   %d\n", points.size());
      for (size_t i = 0; i < slopes.size() - 1; i++) { // 注意长度减一
        center_line.push_back(
            POINT(points[i].x + offsetX, points[i].y + offsetY));

        // printf("-center_line= %d---%d   --%d\n",
        // i,center_line[i].x,center_line[i].y);
      }
    } else {

      for (size_t i = 0; i < points.size() - 1; i++) { // 注意长度减一
        center_line.push_back(
            POINT(points[i].x, 0.5 * (points[i].y + points2[i].y)));
      }
    }


/////////////------------------ 过滤后面位置信息

 vector<POINT> center_line_cal;
 size_t index= center_line.size()-1;
std::vector<POINT>::iterator ptr; // 定义一个迭代器
std::vector<POINT>::iterator start = center_line.begin(); // 记录起始位置
for (ptr = center_line.begin(); ptr != center_line.end(); ptr++) {
    if (ptr->x>800) {
       index = std::distance(start, ptr); // 计算指针位置
        // 输出x值大于500的数据的指针位置
      printf( "\n\n >800 de index= %d",index);
        break;
    }
}
std::vector<POINT> center_line2(center_line.begin(), center_line.begin() + index + 1);
///////////-------------------- 过滤后面位置信息
   ipm_controlCenter= averagepoint(center_line2);

    // 使用平均斜率和中心点计算截距
    return center_line;
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

    // printf("\n\n --ipm-trackRecognition.pointsEdgeRight.size()   =%d\n",
    // trackRecognition.pointsEdgeRight.size());

    // for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
    // {

    //   printf("ipmpointsEdgei=%d- %d=\t%d=\t%d\t %d ---%d\n", i,
    //   ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y,
    //   ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,
    //   ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
    // }

    // printf("\n\n -2-ipm-trackRecognition.pointsEdgeRight.size()   =%d\n",
    // trackRecognition.pointsEdgeRight.size());

    ///////////////////--------------------------

    /////////////////////////-----------------
    //////////////////////-----分割环岛内圆
    int logo = 0, s1 = 0, s2 = 0, cnt = 0;
    for (i = 0; i < ipmpointsEdgeRight.size(); i++) {

      if (ipmpointsEdgeRight[i].y > 318)
        cnt += 1;
      else
        cnt = 0;

      if (cnt > 20)
        break;
    }
    if (cnt > 20) {
      cnt = 0;
      for (j = i; j < ipmpointsEdgeRight.size(); j++) {

        if (ipmpointsEdgeRight[j].y < 280) {
          s1 = j;
          break;
        }
      }
      cnt = 0;
      for (j = s1; j < ipmpointsEdgeRight.size(); j++) {

        if (ipmpointsEdgeRight[j].y < 270)
          cnt += 1;
        else
          cnt = 0;

        if (cnt > 20)
          break;
      }
    }

    if (cnt == 21) {

      for (i = j; i < ipmpointsEdgeRight.size(); i++) {
        if (ipmpointsEdgeRight[i].y > 317) {
          s2 = i - 1;
          break;
        } else if (ipmpointsEdgeRight[i].y < 317 &&
                   i == ipmpointsEdgeRight.size() - 1) {
          s2 = i;
        }
      }

      if (s2 > 0) {
        logo = 1;
      }
    }
    ///////////////////
    printf("--------------Right s1=%d\t s2=%d \n", s1, s2);
    if (logo > 0 && s1 > 0 && s2 - s1 > 20) {

      printf("Right s1=%d\t s2=%d \n", s1, s2);

      for (i = s1; i < s2; i++) {
        circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
               5, colorLine1, 5); // 60行

        circle(dstImg9, Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x),
               5, colorLine2,
               2); // 60行
                
      }

      ///////////////////
      //////////////////////-----分割环岛内圆
      /////////////////////////---------------------------------------------

      // 生成一些二维离散点作为示例数据
      // Mat points(s2-s1+1, 1, CV_32FC2);
      std::vector<cv::Point> points;
      for (int i = s1; i < s2; i++) {
        int x = static_cast<int>(ipmpointsEdgeRight[i].x);
        int y = static_cast<int>(ipmpointsEdgeRight[i].y);
        points.push_back(cv::Point(y, x));
      }
      // 生成点集

      // 检查点集是否至少包含5个点
      if (points.size() < 5) {
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
      float avgRadius = (ellipse.size.width + ellipse.size.height) /
                        4; // 长轴和短轴的平均值除以2
      for (const auto &point : points) {
        double distance =
            cv::norm(cv::Point2f(point.x, point.y) - ellipseCenter);
        error += std::pow(distance - avgRadius, 2);
      }

      printf("Fitting error: %f\n", error);
      printf("Ellipse center: (%f, %f)\n", ellipse.center.x, ellipse.center.y);
      printf("Ellipse average radius: %f\n", avgRadius);
      printf("Ellipse average radius width-height: %3.2f ::\t  %3.2f \n",
             ellipse.size.width * 0.5, ellipse.size.height * 0.5);

      if (avgRadius > 30 && avgRadius < 80 && error < 8000 &&
          abs(ellipse.size.width - ellipse.size.height) < 30)

        ringyes = 1;

      ////////////////////////-------------------------------------
    }
    for (int i = 0; i < ipmpointsEdgeLeft.size(); i = i + 2) {
      circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x), 1,
             colorLine1, 5); // 60行

      circle(dstImg9, Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x),
             1, colorLine2, 2); // 60行

      circle(dstImg9,
             Point(ipmpointsEdgeRight[i].y - 35, ipmpointsEdgeRight[i].x), 1,
             Scalar(0, 0, 255), 2);
     
    }

    //--------------------------------- 读取偏移量
 
    cv::imshow("guan fangku--Transformed", dstImg9);

    return ringyes;
  }


};