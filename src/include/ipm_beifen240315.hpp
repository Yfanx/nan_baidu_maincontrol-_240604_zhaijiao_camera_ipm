#ifndef D929597E_2DB6_4BA3_8BF7_775FD7E4E174
#define D929597E_2DB6_4BA3_8BF7_775FD7E4E174

#include <cmath>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include <vector>


#include "../src/image_preprocess.cpp"
#include "../src/perspective_mapping.cpp"
#include "../src/recognition/track_recognition.cpp" //赛道识别基础类
#include "./common.hpp"                             //公共类方法文件


#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

enum pathstype {
  None = 0,
  pathStraight, // 直入十字
  pathLeftwan,  // 左斜入十字
  pathRightwan, // 右斜入十字

};
pathstype pathstype = pathstype::None;

vector<POINT> pathtypecheck(TrackRecognition &track) {

  vector<POINT> v_center(4); // 三阶贝塞尔曲线
  float heightoffset = 0.5;
  track.validRowsCal();
  printf("---66---Right-track----Left--Right.size()= %d   =%d  \n \n",
         track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());
  //
  printf("-1-concel---validRowsLeft=r--%d-----%d\n", track.validRowsLeft,
         track.validRowsLeftend);
  printf("-1-concel---validRowsRight= r---%d   --%d\n", track.validRowsRight,
         track.validRowsRightend);
  for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
    printf("\n\n-0-con i=-track.pointsEdgeLeft[i].xy=%d -%d --\t %d\n", i,
           track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y); // 绿色点
  }
  int i, j, s1;
  // track.pointsEdgeLeft.assign(track.pointsEdgeLeft.begin() +
  // track.validRowsLeft, track.pointsEdgeLeft.end());

  // track.pointsEdgeRight.assign(track.pointsEdgeRight.begin()+
  // track.validRowsRight, track.pointsEdgeRight.end());

  // 左单边
  if (track.pointsEdgeLeft.size() > ROWSIMAGE * 0.8 &&
      track.pointsEdgeLeft[0].y < track.pointsEdgeLeft.back().y &&
      track.pointsEdgeRight[0].y > track.pointsEdgeRight.back().y &&
      track.stdevEdgeCal2(track.pointsEdgeLeft, track.validRowsLeft,
                          track.pointsEdgeLeft.size() - 1) < 50 &&
      track.stdevEdgeCal2(track.pointsEdgeRight, track.validRowsRight,
                          track.pointsEdgeRight.size() - 1) < 50 &&
      track.pointsEdgeRight.size() > ROWSIMAGE * 0.8 &&
      track.validRowsLeft < 80 && track.validRowsRight < 80) {

    printf(
        "\n\n--vecmax.x=%3.2f \t%3.2f\n", vecmax(track.pointsEdgeLeft),
        vecmax(
            track.pointsEdgeRight)); // 绿色点
                                     // if(vecmax(track.pointsEdgeRight)<320)
                                     // //if(vecmax(track.pointsEdgeRight)<320)
    pathstype = pathstype::pathStraight; // 直入十字None;

  } else {
    s1 = 0;

    if (track.pointsEdgeLeft.size() > 50 &&
        vecmin(track.pointsEdgeRight) > 300) {
      for (j = 2; j < track.pointsEdgeLeft.size() - 5; j++) {
        // cout<<"s1=11111111111"<<endl;

        if (track.pointsEdgeLeft[j + 2].y > track.pointsEdgeLeft[j].y) {
          s1 += 1;
        } else
          s1 = 0;
        if (s1 > 0.4 * (track.pointsEdgeLeft.size() - 10)) {
          pathstype = pathstype::pathRightwan;
          //
          printf("\n\n\n  -----------------pathstype::pathRightwan; \n");

          break;
        }
      }
    }
  }

  // track.validRowsCal();
  // track = retrack(track);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  // track.validRowsRight, track.validRowsRightend);
  return v_center;
  //////////////
}

float ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
{
  // Mat image;
  // image = imread("E:\\Mydevtest\\vscode_opencv\\123.png");

  // TrackRecognition trackRecognition; // 赛道识别
  vector<POINT> pointsEdgeLeftnew;   // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew;  // 赛道右边缘点集
  vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集
  // garageRecognition.disGarageEntry = motionController.params.disGarageEntry;

  printf("--0-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("--1-  pathstype   =%d\n", pathstype);
  // 创建一个2000x600的黑色图像

  // 用于显示的图像

  // 在每一帧的处理开始时

  // 在buffer上进行绘制操作

  cv::Point2f point1, point2;
  int i = 1;
  int startrow = 0;
  int startrow2 = 0;
  // cv::Mat image0= cv::Mat::zeros(800, 500, CV_8UC3);

  // while (1)
  { //

    if (frame0.empty()) {
      cout << "Cannot open image!" << endl;
    }

    // --------------[02] 图像预处理

    // trackRecognition.pointsEdgeLeft.resize(trackRecognition.validRowsLeft);
    // trackRecognition.
    // pointsEdgeRight.resize(trackRecognition.validRowsRight);

    printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
    printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    if (trackRecognition.validRowsLeft > trackRecognition.validRowsRight) {
      startrow = trackRecognition.validRowsLeft;
    } else {
      startrow = trackRecognition.validRowsRight;
    }
    int w = 2000, h = 2000;
    cv::Size size(w, h); // 新尺寸为300x200
    // // 创建一个拷贝用于绘图，避免在原始帧上绘制
    cv::Mat image0; //=imageTrack.clone();
                    //     // 调整图像大小
    cv::resize(frame0, image0, size);
    // cv::namedWindow("imgTrack", WINDOW_NORMAL);

    // imshow("imgTrack", imageTrack);

    printf("-1-concel---validRowsLeft=r--%d-----%d\n",
           trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
    printf("-1-concel---validRowsRight= r---%d   --%d\n",
           trackRecognition.validRowsRight, trackRecognition.validRowsRightend);

    {
      double fx = 257.1156, fy = 257.1485;
      double cx = 120.8040,
             cy = 164.8002; // 注意cx和cy与MATLAB代码中的行列对应关系
      double h = 340.0;
      double a = deg2rad(34);

      // 处理左边的点
      for (auto &point : trackRecognition.pointsEdgeLeft) {
        double u = point.x; // 假设u值存储在x成员中
        double v = point.y; // 假设v值存储在y成员中
        double zc = (h / sin(a))/(1+(u/fx-cx/fx)/tan(a));
        double xc = (u / fx - cx / fx) * zc;
        double yu = (v / fy - cy / fy) * zc;
        double zu = -xc * sin(a) + zc * cos(a);

        // point.z = zu; // 更新点的z值
        // point.u = yu; //
        // 更新点的u值（这里u可能代表不同的意义，根据实际需要调整） POINT
        // pointTmp(zu, yu);

        POINT pointTmp(zu, yu);
        pointsEdgeLeftnew.push_back(pointTmp);
      }

      // 处理右边的点，与左边类似
      for (auto &point : trackRecognition.pointsEdgeRight) {
        double u = point.x;
        double v = point.y;
        double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));

        double xc = (u / fx - cx / fx) * zc;
        double yu = (v / fy - cy / fy) * zc;
        double zu = -xc * sin(a) + zc * cos(a);
        // point.z = zu;
        // point.u = yu;
        // POINT pointTmp(zu, yu);
        POINT pointTmp(zu, yu); // 缩放0.5
        pointsEdgeRightnew.push_back(pointTmp);
      }

      // 这里可以添加绘图或其他处理逻辑

      // printf("pointsEdgeRightnew.size()= %d \n", pointsEdgeRightnew.size());
      // for (i = 2; i < pointsEdgeRightnew.size(); i++)
      // {
      //  //printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i,
      //  trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x,
      //  pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y,
      //  pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);

      //   printf(" %d \t %d\n", pointsEdgeLeftnew[i].x,
      //   pointsEdgeLeftnew[i].y);

      // }
    }
    ///////
    // 假定有两组表示线段的点集，pointsLine1 和 pointsLine2
    // 这里我们手动定义一些示例点
    POINT centermiao;
     float delta;
    vector<POINT> centerEdge;
    int miaorow = 0, py = 0;
    startrow = 0;

    for (int i = startrow; i < pointsEdgeLeftnew.size(); i++) {
      // int py = pointsEdge[i].y + offsetWidth;

      if (pathstype == pathstype::pathRightwan) {
        // printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i,
        // trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x,
        // pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y,
        // pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y); printf(" %d \t
        // %d\n", pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y);
        // POINT pointTmp(zu, yu ); // 缩放0.5
        //   pointsEdgeRightnew[i].x=pointsEdgeLeftnew[i].x;
        py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
      } else {
        py = (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) * 0.5 +
             w * 0.5; // 230707 修给
      }
      centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
      if (pointsEdgeRightnew[i].x > 520 && pointsEdgeRightnew[i].x < 560) {
        miaorow = i;
        centermiao.x = pointsEdgeRightnew[i].x;
        centermiao.y = py - w * 0.5;
      }
    }

    //    for (i = 0; i <centerEdge.size(); i++)
    //             {
    //                 printf("centerEdgei==\t%d=\t%d\t ---%d\n",i,
    //                 centerEdge[i].x, centerEdge[i].y);
    //             }
    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    // // 假设image是你的图像
    // // //
    // //image0 = cv::Mat::zeros(image0.size(), image0.type());startrow
    for (i = pointsEdgeLeftnew.size() - 1; i > 0; i--) {
      circle(
          image0,
          Point(pointsEdgeLeftnew[i].y + 0.5 * w, h - pointsEdgeLeftnew[i].x),
          1, colorLine1, 5); // 60行
      // circle(image0, Point(pointsEdgeRightnew[i].x,
      // pointsEdgeRightnew[i].y+300), 1, Scalar(255, 0, 255), -1); //60行
      circle(
          image0,
          Point(pointsEdgeRightnew[i].y + 0.5 * w, h - pointsEdgeRightnew[i].x),
          1, colorLine2, 2); // 60行
      circle(image0, Point(centerEdge[i].y, centerEdge[i].x), 1,
             cv::Scalar(0, 0, 255), 1); // 60行
    }
    //         // 绘制第一条线

    circle(image0, Point(centerEdge[miaorow].y, centerEdge[miaorow].x), 45,
           cv::Scalar(0, 0, 255), 3); // 60行
    putText(image0,
            "miao: " + formatDoble2String(centermiao.x, 02) + "\t-" +
                formatDoble2String(centermiao.y, 0),
            Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x - 60),
            FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速

    float dx = sqrt(centermiao.x * centermiao.x + centermiao.y * centermiao.y);
   delta = -rad2deg(atan2(2 * 300 * centermiao.y, dx * dx));

    putText(image0, "miao:dx " + formatDoble2String(dx, 0),
            Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x + 80),
            FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速

    putText(image0, "miao:delta " + formatDoble2String(delta, 2),
            Point(centerEdge[miaorow].y - 50, centerEdge[miaorow].x - 100),
            FONT_HERSHEY_PLAIN, 5, Scalar(0, 0, 255), 2); // 车速

    //         cv::imshow("Custom Image with Drawing", image0);
    //         // 按'q'退出
    //         char key = (char) cv::waitKey(30); //
    //         等待30ms，如果按键则返回按键的ASCII码，否则返回-1 if (key == 'q'
    //         || key == 27) { // 27是ESC键的ASCII码
    //             break;
    //         }
    //     }
    // }
    double xielv = centerline_xielv(centerEdge);
    printf("\n\n  xielv= %3.2f \n", xielv);

    printf("\n\nmiaorow centerEdge[miaorow].x-y= %d \t %d \t %d \n", miaorow,
           centerEdge[miaorow].x, centerEdge[miaorow].y);
    printf("\n\n centermiao.x y= %3.2f \t %3.2f \n", centermiao.x,
           centermiao.y);
    if (centerEdge.size() > 5) {
      std::vector<cv::Point2f> points;
      for (auto p : centerEdge) {
        points.emplace_back(cv::Point2f(p.y, p.x));
      }
      cv::Vec4f line;
      cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);

      // Calculate the angle of the line
      double vx = line[0], vy = line[1];

      // 解包Vec4f以获取直线参数
      vx = line[0];
      vy = line[1];
      float x0 = line[2];
      float y0 = line[3];

      // 计算直线上的两点
      // 这里的点选择取决于你想要直线有多长以及图像的尺寸
      // point1(x1, y1) 在直线上，足够远的负方向上
      // point2(x2, y2) 在直线上，足够远的正方向上

      float xie = vx / vy;
      point1.y = centerEdge[0].x;
      // 1000 是一个示例距离，可以根据需要调整

      point1.x = x0 + xie * (point1.y - y0);
      point2.y = centerEdge.back().x;
      point2.x = x0 + xie * (point2.y - y0);
      printf("\n\n point1.xy-point2x.x %3.2f \t %3.2f\t %3.2f \t %3.2f \n",
             point1.x, point1.y, point2.x, point2.y);
      // cv::line(image0,Point( centerEdge[0].y, centerEdge[0].x), Point(
      // centerEdge.back().y, centerEdge.back().x), cv::Scalar(0, 255, 0), 4,
      // cv::LINE_AA);

      // 在图像上画线
      cv::line(image0, point1, point2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    cv::line(image0, Point(centerEdge[miaorow].y, centerEdge[miaorow].x),
             Point(0.5 * w, h), cv::Scalar(0, 120, 255), 2, cv::LINE_AA);
    cv::line(image0, Point(0.5 * w, 1), Point(0.5 * w, h),
             cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    // 显示图像1
    pointsEdgeRightnew.clear();
    pointsEdgeLeftnew.clear();
    centerEdge.clear();
    // 完成所有绘制后，将buffer复制(或交换)到frame
    // destroyAllWindows();

    cv::resize(image0, image0, cv::Size((int)(w * 0.3), (int)(h * 0.3)));

    cv::imshow("Image----", image0);
    ////////////  cv::waitKey(0);

    waitKey(20);
  }

  return delta; // centermiao;
}


#endif                                                                 /* D929597E_2DB6_4BA3_8BF7_775FD7E4E174 */



/////////////////////////////////---------------------------------------------------------240320


#pragma once

#include <cmath>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/types.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include <vector>

#include "../src/image_preprocess.cpp"
#include "../src/perspective_mapping.cpp"
#include "../src/recognition/track_recognition.cpp" //赛道识别基础类
#include "./common.hpp"                             //公共类方法文件

#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

enum pathstype {
  None = 0,
  pathStraight, // 直入十字
  pathLeftwan,  // 左斜入十字
  pathRightwan, // 右斜入十字
  pathRightxie, // 右斜入十字
  pathLeftxie,
};
pathstype pathstype = pathstype::None;

int pathtypecheck(TrackRecognition &track) {

  vector<POINT> v_center(4); // 三阶贝塞尔曲线
  float heightoffset = 0.5;
  // track.validRowsCal();
  // printf("---66---Right-track----Left--Right.size()= %d   =%d  \n \n",
  //        track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());
  // //
  // printf("-1-concel---validRowsLeft=r--%d-----%d\n", track.validRowsLeft,
  //        track.validRowsLeftend);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n", track.validRowsRight,
  //        track.validRowsRightend);

  int i, j, s1;
  // track.pointsEdgeLeft.assign(track.pointsEdgeLeft.begin() +
  // track.validRowsLeft, track.pointsEdgeLeft.end());

  // track.pointsEdgeRight.assign(track.pointsEdgeRight.begin()+
  // track.validRowsRight, track.pointsEdgeRight.end());
  // printf("\n\n\n \n\n\n  ----track.pointsEdgeRight.size() > 50 %d ---%d \n",
  //          track.pointsEdgeRight.size() > 50,
  //          vecmin(track.pointsEdgeRight));
  if (track.pointsEdgeRight.size() > 50 &&
     averagepoint(track.pointsEdgeRight) > 310) {
    s1 = 0;
    for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 5; j++) {
      // cout<<"s1=11111111111"<<endl;

      if (track.pointsEdgeLeft[j + 2].y > track.pointsEdgeLeft[j].y) {
        s1 += 1;
      } else
        s1 = 0;
      if (s1 > 0.2 * (track.pointsEdgeLeft.size() - track.validRowsLeft)) {
        pathstype = pathstype::pathLeftxie; //
        printf("\n\n\n  -----------------pathstype::pathLeftxie;\n");
        return 0;
      }
    }
  }
  // 左单边
  if (track.pointsEdgeLeft.size() > ROWSIMAGE * 0.8 &&
      track.pointsEdgeLeft[0].y < track.pointsEdgeLeft.back().y &&
      track.pointsEdgeRight[0].y > track.pointsEdgeRight.back().y &&
      track.stdevEdgeCal2(track.pointsEdgeLeft, track.validRowsLeft,
                          track.pointsEdgeLeft.size() - 1) < 50 &&
      track.stdevEdgeCal2(track.pointsEdgeRight, track.validRowsRight,
                          track.pointsEdgeRight.size() - 1) < 50 &&
      track.pointsEdgeRight.size() > ROWSIMAGE * 0.8 &&
      track.validRowsLeft < 80 && track.validRowsRight < 80) {

    printf(
        "\n\n--vecmax.x=%3.2f \t%3.2f\n", vecmax(track.pointsEdgeLeft),
        vecmax(
            track.pointsEdgeRight)); // 绿色点
                                     // if(vecmax(track.pointsEdgeRight)<320)
                                     // //if(vecmax(track.pointsEdgeRight)<320)
    pathstype = pathstype::pathStraight; // 直入十字None;
    return 0;
  }

  if (track.pointsEdgeLeft.size() > 50 && vecmin(track.pointsEdgeRight) > 300) {
    s1 = 0;
    for (j = 2; j < track.pointsEdgeLeft.size() - 5; j++) {
      // cout<<"s1=11111111111"<<endl;

      if (track.pointsEdgeLeft[j + 2].y > track.pointsEdgeLeft[j].y) {
        s1 += 1;
      } else
        s1 = 0;
      if (s1 > 0.4 * (track.pointsEdgeLeft.size() - 10)) {
        pathstype = pathstype::pathRightwan;
        //
        printf("\n\n\n  -----------------pathstype::pathRightwan; \n");

        return 0;
      }
    }
  }

  if (track.pointsEdgeLeft.size() > 50 && vecmax(track.pointsEdgeLeft) < 10) {
    s1 = 0;
    for (j = track.validRowsRight; j < track.pointsEdgeRight.size() - 5; j++) {
      // cout<<"s1=11111111111"<<endl;

      if (track.pointsEdgeRight[j + 2].y > track.pointsEdgeRight[j].y) {
        s1 += 1;
      } else
        s1 = 0;
      if (s1 > 0.4 * (track.pointsEdgeRight.size() - track.validRowsRight)) {
        pathstype = pathstype::pathRightxie;
        //
        printf("\n\n\n  -----------------pathstype::pathRightwan; \n");

        return 0;
      }
    }
  }

// 589



  // track.validRowsCal();
  // track = retrack(track);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  // track.validRowsRight, track.validRowsRightend);
  return 0;
  //////////////
}

float ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
{
  vector<POINT> pointsEdgeLeftnew;   // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew;  // 赛道右边缘点集
  vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集
  printf("--0-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("--1-  pathstype   =%d\n", pathstype);
  cv::Point2f point1, point2;
  int i = 1;
  float delta;
  int startrow = 0;
  int startrow2 = 0;
  {
    if (frame0.empty()) {
      cout << "Cannot open image!" << endl;
    }
    // --------------[02] 图像预处理
    printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
    printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

    if (trackRecognition.validRowsLeft > trackRecognition.validRowsRight) {
      startrow = trackRecognition.validRowsLeft;
    } else {
      startrow = trackRecognition.validRowsRight;
    }
    int w = 2000, h = 2000;
    cv::Size size(w, h); // 新尺寸为300x200
    cv::Mat image0;      //=imageTrack.clone();
    cv::resize(frame0, image0, size);
    printf("-1-concel---validRowsLeft=r--%d-----%d\n",
           trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
    printf("-1-concel---validRowsRight= r---%d   --%d\n",
           trackRecognition.validRowsRight, trackRecognition.validRowsRightend);

    {
      double fx = 257.1156, fy = 257.1485;
      double cx = 120.8040,
             cy = 164.8002; // 注意cx和cy与MATLAB代码中的行列对应关系
      double h = 340.0;
      double a = deg2rad(34);
      // 处理左边的点
      for (auto &point : trackRecognition.pointsEdgeLeft) {
        double u = point.x; // 假设u值存储在x成员中
        double v = point.y; // 假设v值存储在y成员中
        double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
        double xc = (u / fx - cx / fx) * zc;
        double yu = (v / fy - cy / fy) * zc;
        double zu = -xc * sin(a) + zc * cos(a);

        POINT pointTmp(zu, yu);
        pointsEdgeLeftnew.push_back(pointTmp);
      }

      // 处理右边的点，与左边类似
      for (auto &point : trackRecognition.pointsEdgeRight) {
        double u = point.x;
        double v = point.y;
        double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));

        double xc = (u / fx - cx / fx) * zc;
        double yu = (v / fy - cy / fy) * zc;
        double zu = -xc * sin(a) + zc * cos(a);
        POINT pointTmp(zu, yu); // 缩放0.5
        pointsEdgeRightnew.push_back(pointTmp);
      }
    }

    POINT centermiao;

    vector<POINT> centerEdge;
    int miaorow = 0, py = 0;
 startrow = 0;

       // startrow =MAX(trackRecognition.validRowsLeft, trackRecognition.validRowsRight);
        
    
    
    if (pathstype == pathstype::pathLeftxie)
    {
      pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
                                   trackRecognition.validRowsLeft,
                               pointsEdgeLeftnew.end());
      float lk = centerline_xielv(pointsEdgeLeftnew);
      printf("\n\n\n\n  lk---= %3.2f \n", lk);
      for (int i = startrow; i < pointsEdgeLeftnew.size(); i++)
      {
        py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
        centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
        if (pointsEdgeRightnew[i].x > 520 && pointsEdgeRightnew[i].x < 560)
        {
          miaorow = i;
          centermiao.x = pointsEdgeRightnew[i].x;
          centermiao.y = py - w * 0.5;
        }
      }
    }


  for (int i = startrow; i < centerEdge.size(); i++)
      {     
       printf("centerEdge[i]i.x-y=--%d--%d--%d \n", i,centerEdge[i].x, centerEdge[i].y);
      }

// if (pathstype == pathstype::pathRightwan) {

//     for (int i = startrow; i < pointsEdgeLeftnew.size(); i++) {

      
//         py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
//       } 
// }
  // else { for (int i = startrow; i < pointsEdgeLeftnew.size(); i++) {

      
  //       py = (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) * 0.5 +
  //            w * 0.5; // 230707 修给
  //     }
  // }
    
 

    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    for (i = pointsEdgeLeftnew.size() - 1; i > startrow; i--) {
      circle(
          image0,
          Point(pointsEdgeLeftnew[i].y + 0.5 * w, h - pointsEdgeLeftnew[i].x),
          1, colorLine1, 5); // 60行

      circle(
          image0,
          Point(pointsEdgeRightnew[i].y + 0.5 * w, h - pointsEdgeRightnew[i].x),
          1, colorLine2, 2); // 60行
    }

     for (int i = startrow; i < centerEdge.size(); i++)
      {    

  circle(image0, Point(centerEdge[i].y, centerEdge[i].x), 2,  cv::Scalar(0, 0, 255), 3); // 60行
           
    }

    circle(image0, Point(centerEdge[miaorow].y, centerEdge[miaorow].x), 45,
           cv::Scalar(0, 0, 255), 3); // 60行
    putText(image0,
            "miao: " + formatDoble2String(centermiao.x, 02) + "\t-" +
                formatDoble2String(centermiao.y, 0),
            Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x - 60),
            FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速

    float dx = sqrt(centermiao.x * centermiao.x + centermiao.y * centermiao.y);
    delta = -rad2deg(atan2(2 * 300 * centermiao.y, dx * dx));

    putText(image0, "miao:dx " + formatDoble2String(dx, 0),
            Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x + 80),
            FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速

    putText(image0, "miao:delta " + formatDoble2String(delta, 2),
            Point(centerEdge[miaorow].y - 50, centerEdge[miaorow].x - 100),
            FONT_HERSHEY_PLAIN, 5, Scalar(0, 0, 255), 2); // 车速

    double xielv = centerline_xielv(centerEdge);
    printf("\n\n  xielv= %3.2f \n", xielv);

    printf("\n\nmiaorow centerEdge[miaorow].x-y= %d \t %d \t %d \n", miaorow,
           centerEdge[miaorow].x, centerEdge[miaorow].y);
    printf("\n\n centermiao.x y= %3.2f \t %3.2f \n", centermiao.x,
           centermiao.y);
    if (centerEdge.size() > 5) {
      std::vector<cv::Point2f> points;
      for (auto p : centerEdge) {
        points.emplace_back(cv::Point2f(p.y, p.x));
      }
      cv::Vec4f line;
      cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
      double vx = line[0], vy = line[1];
      vx = line[0];
      vy = line[1];
      float x0 = line[2];
      float y0 = line[3];
      float xie = vx / vy;
      point1.y = centerEdge[0].x;

      point1.x = x0 + xie * (point1.y - y0);
      point2.y = centerEdge.back().x;
      point2.x = x0 + xie * (point2.y - y0);
      printf("\n\n point1.xy-point2x.x %3.2f \t %3.2f\t %3.2f \t %3.2f \n",
             point1.x, point1.y, point2.x, point2.y);
      // 在图像上画线
      cv::line(image0, point1, point2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
   /* cv::line(image0, Point(centerEdge[miaorow].y, centerEdge[miaorow].x),
             Point(0.5 * w, h), cv::Scalar(0, 120, 255), 2, cv::LINE_AA);
    cv::line(image0, Point(0.5 * w, 1), Point(0.5 * w, h),
             cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    // 显示图像1
    pointsEdgeRightnew.clear();
    pointsEdgeLeftnew.clear();
    centerEdge.clear();
    // 完成所有绘制后，将buffer复制(或交换)到frame
    // destroyAllWindows();

*/
    cv::resize(image0, image0, cv::Size((int)(w * 0.3), (int)(h * 0.3)));

    cv::imshow("Image----", image0);
    ////////////  cv::waitKey(0);

    waitKey(20);
  }

  return delta; // centermiao;
}


/////////////////////////////////--------------------------------------------------------240320

/////////////////////////////////---------------------240323



float ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
{
  vector<POINT> pointsEdgeLeftnew;   // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew;  // 赛道右边缘点集
  vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

  printf("\n\n --00-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("\n\n --11-  pathstype   =%d\n", pathstype);
  cv::Point2f point1, point2;

  int i = 1;
  float delta;
  int startrow = 0;
  int startrow2 = 0;

  if (frame0.empty()) {
    cout << "Cannot open image!" << endl;
  }
  // --------------[02] 图像预处理
  printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
  printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

  // for (int i =0; i < trackRecognition.pointsEdgeRight.size() * 0.95; i++)
  // {

  //   printf("pointsEdgeRight[i].x====== %d-----y=%d \n",
  //   trackRecognition.pointsEdgeRight[i].x,
  //   trackRecognition.pointsEdgeRight[i].y);

  // }

  ///////////////
  // for (int i = 0; i < trackRecognition.pointsEdgeLeft.size() * 0.9;
  //      i++) // 寻找左边跳变点
  // {
  //   printf("trackRecognition.pointsEdgeLeft== %d \t\t x= %d \t y=%d\n",
  //          trackRecognition.pointsEdgeLeft[i].x,
  //          trackRecognition.pointsEdgeLeft[i].y);
  //   // return rowBreakRightUp;
  // }

  ////////////////

  // if (trackRecognition.validRowsLeft > trackRecognition.validRowsRight) {
  //   startrow = trackRecognition.validRowsLeft;
  // } else {
  //   startrow = trackRecognition.validRowsRight;
  // }

  int w = 2000, h = 2000;
  float hpix = 340;
  cv::Size size(w, h); // 新尺寸为300x200
  cv::Mat image0;      //=imageTrack.clone();
  cv::resize(frame0, image0, size);


//////////////////////////////////

        {
            double fx = 257.1156, fy = 257.1485;
            double cx = 120.8040, cy = 164.8002; // 注意cx和cy与MATLAB代码中的行列对应关系
            double h = 340.0;
            double a = deg2rad(34);
            // 处理左边的点
            for (auto &point : trackRecognition.pointsEdgeLeft)
            {
                double u = point.x; // 假设u值存储在x成员中
                double v = point.y; // 假设v值存储在y成员中
                double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
                double xc = (u / fx - cx / fx) * zc;
                double yu = (v / fy - cy / fy) * zc;
                double zu = -xc * sin(a) + zc * cos(a);
                // point.z = zu; // 更新点的z值
                // point.u = yu; // 更新点的u值（这里u可能代表不同的意义，根据实际需要调整）
                // POINT pointTmp(zu, yu);
//  if (trackRecognition.pointsEdgeLeft[i].y < 2)
//       yu =-0.5 * w;
                POINT pointTmp(zu, yu );
                pointsEdgeLeftnew.push_back(pointTmp);
            }

            // 处理右边的点，与左边类似
            for (auto &point : trackRecognition.pointsEdgeRight)
            {
                double u = point.x;
                double v = point.y;
                double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
                double xc = (u / fx - cx / fx) * zc;
                double yu = (v / fy - cy / fy) * zc;
                double zu = -xc * sin(a) + zc * cos(a);

                // point.z = zu;
                // point.u = yu;
                // POINT pointTmp(zu, yu);

      //            if (trackRecognition.pointsEdgeRight[i].y > 318)
      // yu =0.5 * w;
                POINT pointTmp(zu, yu ); // 缩放0.5
                pointsEdgeRightnew.push_back(pointTmp);
            }

            // 这里可以添加绘图或其他处理逻辑

            // printf("pointsEdgeRightnew.size()= %d \n", pointsEdgeRightnew.size());
            // for (i =0; i < pointsEdgeRightnew.size(); i++)
            // {
            //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
            // }
        }

/////////////////////////////


  // printf("-1-concel---validRowsLeft=r--%d-----%d\n",
  //        trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  //        trackRecognition.validRowsRight, trackRecognition.validRowsRightend);
/*
  // 处理右边的点，与左边类似

 

startrow=0;*/
 
  POINT centermiao;
  vector<POINT> centerEdge;
  int miaorow = 0, py = 0;
  startrow = 0;
  // startrow =MAX(trackRecognition.validRowsLeft,
  // trackRecognition.validRowsRight);
  // if (pathstype == pathstype::pathLeftxie)
  // {
  //   pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
  //                                trackRecognition.validRowsLeft,
  //                            pointsEdgeLeftnew.end());
  //   float lk = centerline_xielv(pointsEdgeLeftnew);
  //   printf("\n\n\n\n  lk---= %3.2f \n", lk);

  for (int i =0; i < pointsEdgeLeftnew.size(); i++) {
    py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
    centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
    if (pointsEdgeRightnew[i].x > 520 && pointsEdgeRightnew[i].x < 560) {
      miaorow = i;
      centermiao.x = pointsEdgeRightnew[i].x;
      centermiao.y = py - w * 0.5;
    }
  }

  //}

  // for (int i = startrow; i < centerEdge.size(); i++)
  //     {
  //      printf("centerEdge[i]i.x-y=--%d--%d--%d \n", i,centerEdge[i].x,
  //      centerEdge[i].y);
  //     }

  // if (pathstype == pathstype::pathRightwan) {

  //     for (int i = startrow; i < pointsEdgeLeftnew.size(); i++) {

  //         py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
  //       }
  // }
  // else { for (int i = startrow; i < pointsEdgeLeftnew.size(); i++) {

  //       py = (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) * 0.5 +
  //            w * 0.5; // 230707 修给
  //     }
  // }

 cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
  cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

  for (i = pointsEdgeLeftnew.size() - 1; i > startrow; i--) {
    circle(image0,
           Point(pointsEdgeLeftnew[i].y + 0.5 * w, h - pointsEdgeLeftnew[i].x),
           1, colorLine1, 5); // 60行

    circle(
        image0,
        Point(pointsEdgeRightnew[i].y + 0.5 * w, h - pointsEdgeRightnew[i].x),
        1, colorLine2, 2); // 60行

  circle(image0, Point(centerEdge[i].y, centerEdge[i].x), 2,  cv::Scalar(0,
  0, 255), 3); // 60行

  }

 

  // circle(image0, Point(centerEdge[miaorow].y, centerEdge[miaorow].x), 45,
  //        cv::Scalar(0, 0, 255), 3); // 60行
  // putText(image0,
  //         "miao: " + formatDoble2String(centermiao.x, 02) + "\t-" +
  //             formatDoble2String(centermiao.y, 0),
  //         Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x - 60),
  //         FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速

  // float dx = sqrt(centermiao.x * centermiao.x + centermiao.y * centermiao.y);
  // delta = -rad2deg(atan2(2 * 300 * centermiao.y, dx * dx));

  // putText(image0, "miao:dx " + formatDoble2String(dx, 0),
  //         Point(centerEdge[miaorow].y - 150, centerEdge[miaorow].x + 80),
  //         FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速

  // putText(image0, "miao:delta " + formatDoble2String(delta, 2),
  //         Point(centerEdge[miaorow].y - 50, centerEdge[miaorow].x - 100),
  //         FONT_HERSHEY_PLAIN, 5, Scalar(0, 0, 255), 2); // 车速

  // double xielv = centerline_xielv(centerEdge);
  // printf("\n\n  xielv= %3.2f \n", xielv);

  // printf("\n\nmiaorow centerEdge[miaorow].x-y= %d \t %d \t %d \n", miaorow,
  //        centerEdge[miaorow].x, centerEdge[miaorow].y);
  // printf("\n\n centermiao.x y= %3.2f \t %3.2f \n", centermiao.x,
  //        centermiao.y);

  cv::resize(image0, image0, cv::Size((int)(w * 0.4), (int)(h * 0.4)));
  ///////////////////------------------
  string imageName = "your_image_name.jpg";
  MouseParams params;
  params.img = image0;
  params.imageName = imageName;
  namedWindow("Image");
  setMouseCallback("Image", onMouse, &params);
  imshow("Image", image0);
 // waitKey(20); // 如果不是0，则没法在图片上打印坐标值
  ///////////////////------------------ /**/
  return delta; // centermiao;
}
//--------------------------240323

//==============240325

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


enum pathstype {
  None = 0,
  pathStraight, // 直入十字
  pathLeftwan,  // 左斜入十字
  pathRightwan, // 右斜入十字
  pathRightxie, // 右斜入十字
  pathLeftxie,
};

pathstype pathstype = pathstype::None;
  vector<POINT> pointsEdgeLeftnew;   // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew;  // 赛道右边缘点集
  vector<POINT> centerEdge, centerEdgereal;

Mat imageoriginal;
int pathtypecheck(TrackRecognition &track) {

 pathstype = pathstype::None;
  vector<POINT> v_center(4); // 三阶贝塞尔曲线
  float heightoffset = 0.5;
 
  int i, j, s1;
 
  if (track.pointsEdgeRight.size() > 50 &&
     averagepoint(track.pointsEdgeRight) > 310) {
    s1 = 0;
    for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 5; j++) {
      // cout<<"s1=11111111111"<<endl;

      if (track.pointsEdgeLeft[j + 2].y > track.pointsEdgeLeft[j].y) {
        s1 += 1;
      } else
        s1 = 0;
      if (s1 > 0.2 * (track.pointsEdgeLeft.size() - track.validRowsLeft)) {
        pathstype = pathstype::pathLeftxie; //
        printf("\n\n\n  -----------------pathstype::pathLeftxie;\n");
        return 0;
      }
    }
  }




  //track.validRowsLeft <100 && track.validRowsRight <100 && straight path
  if (vecmin(track.pointsEdgeLeft, track.validRowsLeft)>0&& vecmax(track.pointsEdgeRight, track.validRowsRight)<319) {

    printf("\n\n--vecmax.x=%3.2f \t%3.2f\n", vecmin(track.pointsEdgeLeft, track.validRowsLeft),
        vecmax(track.pointsEdgeRight, track.validRowsRight)); // 绿色点
                                     // if(vecmax(track.pointsEdgeRight)<320)
                                     // //if(vecmax(track.pointsEdgeRight)<320)
    pathstype = pathstype::pathStraight; // 直入十字None;
    return 0;
  }
//track.validRowsLeft <100 && track.validRowsRight <100 &&




  if (vecmin(track.pointsEdgeLeft, track.validRowsLeft)>0&& vecmax(track.pointsEdgeRight, track.validRowsRight) > 315) {
    s1 = 0;
    // for (j = 2; j < track.pointsEdgeLeft.size() - 5; j++) {
    //   // cout<<"s1=11111111111"<<endl;

    //   if (track.pointsEdgeLeft[j + 2].y > track.pointsEdgeLeft[j].y) {
    //     s1 += 1;
    //   } else
    //     s1 = 0;
    //   if (s1 > 0.4 * (track.pointsEdgeLeft.size() - 10)) 
      {
        pathstype = pathstype::pathRightwan;
        //
        printf("\n\n\n  -----------------pathstype::pathRightwan; \n");

        return 0;
      }
    }


  if (track.pointsEdgeLeft.size() >20 && vecmax(track.pointsEdgeLeft,track.validRowsLeft) < 10&& vecmax(track.pointsEdgeRight, track.validRowsRight)<319) {
    s1 = 0;
    // for (j = track.validRowsRight; j < track.pointsEdgeRight.size() - 5; j++) {
    //   // cout<<"s1=11111111111"<<endl;

    //   if (track.pointsEdgeRight[j + 2].y > track.pointsEdgeRight[j].y) {
    //     s1 += 1;
    //   } else
    //     s1 = 0;
    //   if (s1 > 0.4 * (track.pointsEdgeRight.size() - track.validRowsRight))
     {
        pathstype = pathstype::pathRightxie;
        //
        printf("\n\n\n  -----------------pathstype::pathRightwan; \n");

        return 0;
      }
    }
 

// 589



  // track.validRowsCal();
  // track = retrack(track);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  // track.validRowsRight, track.validRowsRightend);
  return 0;
  //////////////
}




void  ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
{
imageoriginal=frame0.clone();
  // vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  // vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

  printf("\n\n --00-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("\n\n --11-  pathstype   =%d\n", pathstype);
  cv::Point2f point1, point2;

  int i = 1;
  float delta;
  int startrow = 0;
  int startrow2 = 0;

  if (frame0.empty()) {
    cout << "Cannot open image!" << endl;
  }
  // --------------[02] 图像预处理
  printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
  printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

 
  int w = 2000, h = 2000;
  float hpix = 340;
  cv::Size size(w, h); // 新尺寸为300x200
  cv::Mat image1,image0;      //
  cv::resize(imageoriginal, image0, size);


//////////////////////////////////  image0=image1.clone();

        {
            double fx = 257.1156, fy = 257.1485;
            double cx = 120.8040, cy = 164.8002;   // 注意cx和cy与MATLAB代码中的行列对应关系
            double h = 340.0;
            double a = deg2rad(34);
            // 处理左边的点
            for (auto &point : trackRecognition.pointsEdgeLeft)
            {
                double u = point.x; // 假设u值存储在x成员中
                double v = point.y; // 假设v值存储在y成员中
                double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
                double xc = (u / fx - cx / fx) * zc;
                double yu = (v / fy - cy / fy) * zc;
                double zu = -xc * sin(a) + zc * cos(a);
                POINT pointTmp(zu, yu );
                pointsEdgeLeftnew.push_back(pointTmp);
            }

            // 处理右边的点，与左边类似
            for (auto &point : trackRecognition.pointsEdgeRight)
            {
                double u = point.x;
                double v = point.y;
                double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
                double xc = (u / fx - cx / fx) * zc;
                double yu = (v / fy - cy / fy) * zc;
                double zu = -xc * sin(a) + zc * cos(a);
                POINT pointTmp(zu, yu ); // 缩放0.5
                pointsEdgeRightnew.push_back(pointTmp);
            }

            // 这里可以添加绘图或其他处理逻辑

            // printf("pointsEdgeRightnew.size()= %d \n", pointsEdgeRightnew.size());
            // for (i =0; i < pointsEdgeRightnew.size(); i++)
            // {
            //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
            // }
        }

/////////////////////////////


  // printf("-1-concel---validRowsLeft=r--%d-----%d\n",
  //        trackRecognition.validRowsLeft, trackRecognition.validRowsLeftend);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  //        trackRecognition.validRowsRight, trackRecognition.validRowsRightend);
/*
  // 处理右边的点，与左边类似



startrow=0;*/
if (  centerEdgereal.empty()) {
        std::cout << "The vector  centerEdgereal is empty." << std::endl;
    } else {
        std::cout << "The vector  centerEdgereal is not empty." << std::endl;
    }

  centerEdgereal.clear();


if (  centerEdgereal.empty()) {
        std::cout << "-----The vector  centerEdgereal is empty." << std::endl;
    } else {
        std::cout << "-----The vector  centerEdgereal is not empty." << std::endl;
    }


  POINT centermiao;

  int miaorow = 0, py = 0,pyreal=0;
  startrow = 0;
  // startrow =MAX(trackRecognition.validRowsLeft,
  // trackRecognition.validRowsRight);
  // if (pathstype == pathstype::pathLeftxie)
  // {
  //   pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
  //                                trackRecognition.validRowsLeft,
  //                            pointsEdgeLeftnew.end());
  //   float lk = centerline_xielv(pointsEdgeLeftnew);
  //   printf("\n\n\n\n  lk---= %3.2f \n", lk); 
  

  for (int i = 0; i < pointsEdgeLeftnew.size(); i++)
  {
    if (pathstype == pathstype::pathStraight)
    {
      py = 0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) + w * 0.5;
      pyreal=0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y);
    }
    else if (pathstype == pathstype::pathRightwan||pathstype == pathstype::pathLeftxie)
    {
      py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
      pyreal=pointsEdgeLeftnew[i].y +225;
    }

    else if (pathstype == pathstype::pathLeftwan||pathstype ==pathstype::pathRightxie)
    {

        py = pointsEdgeRightnew[i].y-225 + w * 0.5;
        pyreal=pointsEdgeRightnew[i].y -225;
    }


centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
centerEdgereal.emplace_back(pointsEdgeRightnew[i].x,pyreal);


  }

  // for (int i = 0; i < centerEdge.size(); i++)
  //     {
  //      printf("centerEdge[i]i.x-y=   %d  %d   %d   %d \n",centerEdge[i].x,centerEdgereal[i].x,
  //      centerEdge[i].y,centerEdgereal[i].y);
  //     }

 

 
void  ipm_world(TrackRecognition &trackRecognition, Mat &frame0) //
{
imageoriginal=frame0.clone();
  // vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  // vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

  //printf("\n\n --00-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("\n\n --11-  pathstype   =%d\n", pathstype);
  cv::Point2f point1, point2;

  int i = 1;
  float delta;
  int startrow = 0;
  int startrow2 = 0;

  if (frame0.empty()) {
    cout << "Cannot open image!" << endl;
  }
  // --------------[02] 图像预处理
  // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
  // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

 
  int w = 2000, h = 2000;
  float hpix = 340;
  cv::Size size(w, h); // 新尺寸为300x200
  cv::Mat image1,image0;      //
  cv::resize(imageoriginal, image0, size);


//////////////////////////////////  image0=image1.clone();

        {
            double fx = 257.29, fy = 257.01;
            double cx = 119.894, cy = 158.05755;   // 注意cx和cy与MATLAB代码中的行列对应关系
            double ch = 338;
            double a = deg2rad(33.69);
            // 处理左边的点
         
      for (auto &point : trackRecognition.pointsEdgeLeft) {
        double u = point.x; // 假设u值存储在x成员中
        double v = point.y; // 假设v值存储在y成员中
        double zc = (ch / sin(a))/(1+(u/fx-cx/fx)/tan(a));
        double xc = (u / fx - cx / fx) * zc;
        double yu = (v / fy - cy / fy) * zc;
        double zu = -xc * sin(a) + zc * cos(a);

        // point.z = zu; // 更新点的z值
        // point.u = yu; //
        // 更新点的u值（这里u可能代表不同的意义，根据实际需要调整） POINT
        // pointTmp(zu, yu);

        POINT pointTmp(zu, yu);
        pointsEdgeLeftnew.push_back(pointTmp);
      }

      // 处理右边的点，与左边类似
      for (auto &point : trackRecognition.pointsEdgeRight) {
        double u = point.x;
        double v = point.y;
        double zc = (ch / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));

        double xc = (u / fx - cx / fx) * zc;
        double yu = (v / fy - cy / fy) * zc;
        double zu = -xc * sin(a) + zc * cos(a);
        // point.z = zu;
        // point.u = yu;
        // POINT pointTmp(zu, yu);
        POINT pointTmp(zu, yu); // 缩放0.5
        pointsEdgeRightnew.push_back(pointTmp);
      }

            // 这里可以添加绘图或其他处理逻辑

            printf("pointsEdgeRightnew.size()= %d \n", pointsEdgeRightnew.size());
            for (i =0; i < pointsEdgeRightnew.size(); i++)
            {
             printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
            }
        }

/////////////////////////////


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

  int miaorow = 0, py = 0,pyreal=0;
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
printf("\n\n\n\n  endrow---= %d   %d \n",endrow,trackRecognition.pointsEdgeRight.size()); 
  // for (int i = 0; i <endrow; i++)

 for (int i = 0; i <trackRecognition.pointsEdgeRight.size(); i++)
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


  }

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


 

 
 

  cv::resize(image0, image0, cv::Size((int)(w *0.25), (int)(h * 0.25)));
  ///////////////////------------------
  string imageName = "your_image_name.jpg";
  MouseParams params;
  params.img = image0;
  params.imageName = imageName;
  namedWindow("Image");
  setMouseCallback("Image", onMouse, &params);



 putText(image0, "xy0-: " + formatDoble2String(centerEdgereal[0].x, 0)+"   " +formatDoble2String(centerEdgereal[0].y,0),
          Point(20, h*0.2 ),FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速


 putText(image0, "xy-end: " + formatDoble2String(centerEdgereal.back().x, 0)+"   " +formatDoble2String(centerEdgereal.back().y,0),
          Point(20, h*0.02 ),FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2); // 车速


  Scalar color(255, 0, 255);    // 线条颜色（BGR格式）
    int thickness =1;   

line(image0, Point(250, 0),Point(250, 499),  Scalar(255, 255, 0) , 2);

line(image0, Point(1, 100),Point(499, 100), color, thickness);  //200
line(image0, Point(1, 200),Point(499, 200), color, thickness);  //200
line(image0, Point(1, 300),Point(499, 300), color, thickness);  //200

line(image0, Point(1, 400),Point(499, 400), color, thickness);  //200
line(image0, Point(1, 450),Point(499, 450), color, thickness);  //200

//------------------------------------图片上一定要加，否则会重复上一张线！！！！！！！！！
// pointsEdgeRightnew.clear();
// pointsEdgeLeftnew.clear();
// centerEdge.clear();
//centerEdgereal.clear();
//-----------------------------------图片上一定要加，否则会重复上一张线！！！！！！！！！



//imshow("Image", image0);


  
  // waitKey(20); 
  
 // cv::destroyWindow("Image");
  // 如果不是0，则没法在图片上打印坐标值
  ///////////////////------------------ /**/
  //return centerEdgereal; // centermiao;
}
};



//============240325


//---------------------240326

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


enum pathstype {
  None = 0,
  pathStraight, // 直入十字
  pathLeftwan,  // 左斜入十字
  pathRightwan, // 右斜入十字
  pathRightxie, // 右斜入十字
  pathLeftxie,
};

pathstype pathstype = pathstype::None;
  vector<POINT> pointsEdgeLeftnew;   // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew;  // 赛道右边缘点集
  vector<POINT> centerEdge, centerEdgereal;

Mat imageoriginal;
int pathtypecheck(TrackRecognition &track) {

 pathstype = pathstype::None;
  vector<POINT> v_center(4); // 三阶贝塞尔曲线
  float heightoffset = 0.5;
 
  int i, j, s1;
 
//////////////////-----------pathstype == pathstype::pathLeftwan-----------------------------

  // if (track.validRowsLeft>0.8*track.pointsEdgeLeft.size()&&track.validRowsRight<0.6*track.pointsEdgeRight.size()&&track.pointsEdgeRight.size() > 50 &&
  //    averagepoint(track.pointsEdgeLeft)<30) 
     
     
  //      if (track.pointsEdgeRight.size()>50&&track.validRowsRight<0.6*track.pointsEdgeRight.size()&&vecmin(track.pointsEdgeLeft, track.validRowsLeft)<1&& vecmax(track.pointsEdgeRight, track.validRowsRight)<319)
  //    {
  //   s1 = 0;
  //   for (j =track.validRowsRight; j < track.pointsEdgeRight.size() - 5; j++) {
  //     // cout<<"s1=11111111111"<<endl;

  //     if (track.pointsEdgeRight[j +4].y<track.pointsEdgeRight[j].y) {
  //       s1 += 1;
  //     } else
  //     { s1 = 0;break; }

  //     if (s1 >0) 
  //     {
  //       pathstype =pathstype::pathLeftwan; //
  //       printf("\n\n\n  -----------------pathstype == pathstype::pathLeftwan;\n");
  //       return 0;
  //     }
  //   }
  // }




////////////////////------------pathstype == pathstype::pathLeftwan-----------------




////////////////////-----------pathstype = pathstype::pathLeftxie-----------------

  // if (track.pointsEdgeRight.size() > 50 &&
  //    averagepoint(track.pointsEdgeRight) > 310) {
  //   s1 = 0;
  //   for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 5; j++) {
  //     // cout<<"s1=11111111111"<<endl;

  //     if (track.pointsEdgeLeft[j + 2].y > track.pointsEdgeLeft[j].y) {
  //       s1 += 1;
  //     } else
  //       s1 = 0;
  //     if (s1 > 0.2 * (track.pointsEdgeLeft.size() - track.validRowsLeft)) {
  //       pathstype = pathstype::pathLeftxie; //
  //       printf("\n\n\n  -----------------pathstype::pathLeftxie;\n");
  //       return 0;
  //     }
  //   }
  // }



////////////////////-----------pathstype = pathstype::pathLeftxie-----------------





////////////////////---------- pathstype = pathstype::pathStraight-----------------

  //track.validRowsLeft <100 && track.validRowsRight <100 && straight path
  if (track.validRowsLeft<0.6*track.pointsEdgeLeft.size()&&track.validRowsRight<0.6*track.pointsEdgeRight.size()&&sigma_valid(track.pointsEdgeLeft, track.validRowsLeft)<1000&&sigma_valid(track.pointsEdgeRight, track.validRowsRight)<1000&&vecmin(track.pointsEdgeLeft, track.validRowsLeft)>0&& vecmax(track.pointsEdgeRight, track.validRowsRight)<319) {

    printf("\n\n-pathstype::pathStraighttrack.validRowsLeft-vecmax.x=%3.2f \t %3.2f\t  %d\n",sigma_valid(track.pointsEdgeLeft, track.validRowsLeft),  sigma_valid(track.pointsEdgeRight, track.validRowsRight),vecmax(track.pointsEdgeRight, track.validRowsRight)); // 绿色点
                                     // if(vecmax(track.pointsEdgeRight)<320)
                                     // //if(vecmax(track.pointsEdgeRight)<320)
    pathstype = pathstype::pathStraight; // 直入十字None;

//////////
//   for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size(); j++) 
//  printf("-ptrack.pointsEdgeLeft=%d \t%d\n", track.pointsEdgeLeft[j].x, track.pointsEdgeLeft[j].y);
////////////
    return 0;
  }
//track.validRowsLeft <100 && track.validRowsRight <100 &&

////////////////////---------- pathstype = pathstype::pathStraight-----------------






////////////////////---------- pathstype = pathstype::pathRightwan----------------

  if (track.pointsEdgeLeft.size()>20&&vecmin(track.pointsEdgeLeft, track.validRowsLeft)>0&& vecmax(track.pointsEdgeRight, track.validRowsRight) > 315) {
    s1 = 0;
    for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size() - 5; j++) {
      // cout<<"s1=11111111111"<<endl;

      if (track.pointsEdgeLeft[j + 4].y > track.pointsEdgeLeft[j].y) {
        s1 += 1;
      } else
        {s1 = 0;
        break;}

      if (s1 >0) 
      {
        pathstype = pathstype::pathRightwan;
        //
        printf("\n\n\n  -----------------pathstype::pathRightwan; \n");

        return 0;
      }
    }
  }

////////////////////---------- pathstype = pathstype::pathRightwan----------------



  if ( track.pointsEdgeRight.size() >20 && vecmax(track.pointsEdgeLeft,track.validRowsLeft) < 10&& vecmax(track.pointsEdgeRight, track.validRowsRight)<319) {
    s1 = 0;
    for (j = track.validRowsRight; j < track.pointsEdgeRight.size() - 5; j++) {
      // cout<<"s1=11111111111"<<endl;

      if (track.pointsEdgeRight[j +4].y<track.pointsEdgeRight[j].y) {
        s1 += 1;
      } else
        s1 = 0;
      if (s1 >0)
     {
        pathstype = pathstype::pathRightxie;
        printf("\n\n\n  -----------------pathstype::pathRightxie; \n");
        return 0;
      }
    }
  }
 

// 589



  // track.validRowsCal();
  // track = retrack(track);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  // track.validRowsRight, track.validRowsRightend);
  return 0;
  //////////////
}




void  ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
{
imageoriginal=frame0.clone();
  // vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  // vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

  //printf("\n\n --00-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("\n\n --11-  pathstype   =%d\n", pathstype);
  cv::Point2f point1, point2;

  int i = 1;
  float delta;
  int startrow = 0;
  int startrow2 = 0;

  if (frame0.empty()) {
    cout << "Cannot open image!" << endl;
  }
  // --------------[02] 图像预处理
  // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
  // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

 
  int w = 2000, h = 2000;
  float hpix = 340;
  cv::Size size(w, h); // 新尺寸为300x200
  cv::Mat image1,image0;      //
  cv::resize(imageoriginal, image0, size);


//////////////////////////////////  image0=image1.clone();

        {
            double fx = 257.1156, fy = 257.1485;
            double cx = 120.8040, cy = 164.8002;   // 注意cx和cy与MATLAB代码中的行列对应关系
            double h = 340.0;
            double a = deg2rad(34);
            // 处理左边的点
            for (auto &point : trackRecognition.pointsEdgeLeft)
            {
                double u = point.x; // 假设u值存储在x成员中
                double v = point.y; // 假设v值存储在y成员中
                double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
                double xc = (u / fx - cx / fx) * zc;
                double yu = (v / fy - cy / fy) * zc;
                double zu = -xc * sin(a) + zc * cos(a);
                POINT pointTmp(zu, yu );
                pointsEdgeLeftnew.push_back(pointTmp);
            }

            // 处理右边的点，与左边类似
            for (auto &point : trackRecognition.pointsEdgeRight)
            {
                double u = point.x;
                double v = point.y;
                double zc = (h / sin(a)) / (1 + (u / fx - cx / fx) / tan(a));
                double xc = (u / fx - cx / fx) * zc;
                double yu = (v / fy - cy / fy) * zc;
                double zu = -xc * sin(a) + zc * cos(a);
                POINT pointTmp(zu, yu ); // 缩放0.5
                pointsEdgeRightnew.push_back(pointTmp);
            }

            // 这里可以添加绘图或其他处理逻辑

            // printf("pointsEdgeRightnew.size()= %d \n", pointsEdgeRightnew.size());
            // for (i =0; i < pointsEdgeRightnew.size(); i++)
            // {
            //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
            // }
        }

/////////////////////////////


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

  int miaorow = 0, py = 0,pyreal=0;
  startrow = 0;
  // startrow =MAX(trackRecognition.validRowsLeft,
  // trackRecognition.validRowsRight);
  // if (pathstype == pathstype::pathLeftxie)
  // {
  //   pointsEdgeLeftnew.assign(pointsEdgeLeftnew.begin() +
  //                                trackRecognition.validRowsLeft,
  //                            pointsEdgeLeftnew.end());
  //   float lk = centerline_xielv(pointsEdgeLeftnew);
  //   printf("\n\n\n\n  lk---= %3.2f \n", lk); 
  

  for (int i = 0; i < pointsEdgeLeftnew.size(); i++)
  {
    if (pathstype == pathstype::pathStraight)
    {
      py = 0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y) + w * 0.5;
      pyreal=0.5 * (pointsEdgeLeftnew[i].y + pointsEdgeRightnew[i].y);
    }
    else if (pathstype == pathstype::pathRightwan||pathstype == pathstype::pathLeftxie)
    {
      py = pointsEdgeLeftnew[i].y + 225 + w * 0.5;
      pyreal=pointsEdgeLeftnew[i].y +225;
    }

    else if (pathstype == pathstype::pathLeftwan||pathstype ==pathstype::pathRightxie)
    {

        py = pointsEdgeRightnew[i].y-225 + w * 0.5;
        pyreal=pointsEdgeRightnew[i].y -225;
    }


centerEdge.emplace_back(h - pointsEdgeRightnew[i].x, py);
centerEdgereal.emplace_back(pointsEdgeRightnew[i].x,pyreal);


  }

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
};


//--------------------------240330



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

pathstype pathstype = pathstype::None;
  vector<POINT> pointsEdgeLeftnew;   // 赛道左边缘点集
  vector<POINT> pointsEdgeRightnew;  // 赛道右边缘点集
  vector<POINT> centerEdge, centerEdgereal;

Mat imageoriginal;

int endrow=100;



int pathtypecheck(TrackRecognition &track) {

 pathstype = pathstype::None;
  vector<POINT> v_center(4); // 三阶贝塞尔曲线
  float heightoffset = 0.5;
 
  int i, j, s1,k1;
 
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

        printf("-0-concel-length--validRowsLeft=r--%d-- %d  ---%d\n" ,track.pointsEdgeLeft.size(), track.validRowsLeft, track.validRowsLeftend);

        printf("-0-concel--length---validRowsRight= r---%d  ---- %d  --%d\n",  track.pointsEdgeRight.size(),track.validRowsRight, track.validRowsRightend);
       
   if (track.pointsEdgeRight.size() >50&&(track.pointsEdgeRight[track.validRowsRight].x - track.pointsEdgeLeft[track.validRowsLeft].x) > 120)    
  {
    s1 = 0;k1=0;
    for (j = track.validRowsRight; j < track.pointsEdgeRight.size()-5; j++) {
      // cout<<"s1=11111111111"<<endl;
  //printf(" track.pointsEdgeLeft ---------%d  %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);
      if (track.pointsEdgeRight[j + 5].y<track.pointsEdgeRight[j].y) {
        s1 += 1;
        k1+=1;
      } else
      {
         s1 = 0;
         break;
      }
    }
       
      if (s1<1&&j-track.validRowsRight>10||k1>50) {

  track.pointsEdgeLeft.resize(j);
   track.pointsEdgeRight.resize(j);
  
endrow=j;
       //
       pathstype =pathstype::pathRightxie;
        printf("\n\n\n  ok-----------------pathstype::pathRightxie\n"); 
        printf("------Left.size()-----%d  %d---%d  %d\n",track.validRowsLeft,track.pointsEdgeLeft.size(),track.validRowsRight,track.pointsEdgeRight.size());


        return 0;
      }
    }
    
////////////////////------------pathstype == pathstype:: pathRightxie-----------------



//////////////////-----------pathstype == pathstype:: pathLeftxie-----------------------------

  //printf("track.validRowsLeft<0.7*track.pointsEdgeLeft.size()---------%d  %d\n",track.validRowsLeft,track.pointsEdgeLeft.size());

   if (track.pointsEdgeLeft.size() >50&&(track.pointsEdgeLeft[track.validRowsLeft].x - track.pointsEdgeRight[track.validRowsRight].x) > 120)    
  {
    s1 = 0;
    for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size()-5; j++) {
      // cout<<"s1=11111111111"<<endl;
  //printf(" track.pointsEdgeLeft ---------%d  %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);
      if (track.pointsEdgeLeft[j + 5].y > track.pointsEdgeLeft[j].y) {
        s1 += 1;
      } else
      {
         s1 = 0;
         break;
      }
    }
       
      if (s1<1&&j-track.validRowsLeft>10) {

  track.pointsEdgeLeft.resize(j);
   track.pointsEdgeRight.resize(j);
  

       pathstype =pathstype::pathLeftxie;
        printf("\n\n\n  ok-----------------pathstype::pathLeftedge\n");
        return 0;
      }
    }
    
////////////////////------------pathstype == pathstype:: pathLeftxie-----------------



        track.validRowsCal();



        printf("-1-concel-length--validRowsLeft=r--%d-- %d  ---%d\n" ,track.pointsEdgeLeft.size(), track.validRowsLeft, track.validRowsLeftend);

        printf("-1-concel--length---validRowsRight= r---%d  ---- %d  --%d\n",  track.pointsEdgeRight.size(),track.validRowsRight, track.validRowsRightend);
       

//////












////////////////////---------- pathstype = pathstype::pathStraight-----------------

  if (track.validRowsLeft<0.6*track.pointsEdgeLeft.size()&&track.validRowsRight<0.6*track.pointsEdgeRight.size()&&sigma_valid(track.pointsEdgeLeft, track.validRowsLeft)<1000&&sigma_valid(track.pointsEdgeRight, track.validRowsRight)<1000&&vecmin(track.pointsEdgeLeft, track.validRowsLeft)>0&& vecmax(track.pointsEdgeRight, track.validRowsRight)<319) {

    printf("\n\n-pathstype::pathStraighttrack.validRowsLeft-vecmax.x=%3.2f \t %3.2f\t  %d\n",sigma_valid(track.pointsEdgeLeft, track.validRowsLeft),  sigma_valid(track.pointsEdgeRight, track.validRowsRight),vecmax(track.pointsEdgeRight, track.validRowsRight)); // 绿色点                      
    pathstype = pathstype::pathStraight; // 直入十字None;
    return 0;
  }
////////////////////---------- pathstype = pathstype::pathStraight-----------------

























//////////////////-----------pathstype == pathstype::pathLeftedge-----------------------------

  //printf("track.validRowsLeft<0.7*track.pointsEdgeLeft.size()---------%d  %d\n",track.validRowsLeft,track.pointsEdgeLeft.size());

  if (track.pointsEdgeLeft.size()>50&&vecmin(track.pointsEdgeLeft, track.validRowsLeft)>0&&track.validRowsLeft<0.7*track.pointsEdgeLeft.size())     
  {
    s1 = 0;
    for (j = track.validRowsLeft; j < track.pointsEdgeLeft.size()-5; j++) {
      // cout<<"s1=11111111111"<<endl;
  //printf(" track.pointsEdgeLeft ---------%d  %d\n",track.pointsEdgeLeft[j].x,track.pointsEdgeLeft[j].y);
      if (track.pointsEdgeLeft[j + 5].y > track.pointsEdgeLeft[j].y) {
        s1 += 1;
      } else
      {
         s1 = 0;
         break;
      }
    }
       
      if (s1 >0) {
       pathstype =pathstype::pathLeftedge;
        printf("\n\n\n  ok-----------------pathstype::pathLeftedge\n");
        return 0;
      }
    }


////////////////////------------pathstype == pathstype::pathLeftedge-----------------








//  for (j = 0; j < track.pointsEdgeRight.size(); j++) 
//       // cout<<"s1=11111111111"<<endl;
//   printf(" track.pointsEdgeLeft ---------%d  %d   %d\n",track.pointsEdgeRight[j].x,track.pointsEdgeLeft[j].y,track.pointsEdgeRight[j].y);





////////////////////---------- pathstype = pathstype::pathRightedge----------------

    if (track.pointsEdgeRight.size() > 50 && track.validRowsRight < 0.8 * track.pointsEdgeRight.size() && vecmax(track.pointsEdgeRight, track.validRowsRight) < 319)
    {
      s1 = 0;
      for (j = track.validRowsRight; j < track.pointsEdgeRight.size() - 12; j++)
      {
        // cout<<"s1=11111111111"<<endl;

        if (track.pointsEdgeRight[j +10].y < track.pointsEdgeRight[j].y)
        {
          s1 += 1;
        }
        else
        {
          s1 = 0;
          break;
        }
      }
      if (s1 > 0)
      {
        pathstype = pathstype::pathRightedge; //
        printf("\n\n\n  -----------------pathstype ==pathstype::pathRightedge;\n");
        return 0;
      }
    }

////////////////////---------- pathstype = pathstype::pathRightedge----------------



  // track.validRowsCal();
  // track = retrack(track);
  // printf("-1-concel---validRowsRight= r---%d   --%d\n",
  // track.validRowsRight, track.validRowsRightend);
  return 0;
  //////////////
}




void  ipm_miaodian(TrackRecognition &trackRecognition, Mat &frame0) //
{
imageoriginal=frame0.clone();
  // vector<POINT> pointsEdgeLeftnew2;  // 赛道左边缘点集
  // vector<POINT> pointsEdgeRightnew2; // 赛道右边缘点集

  //printf("\n\n --00-  pathstype   =%d\n", pathstype);

  // pathstype =pathstype::pathStraight // 直入十字None;
  pathtypecheck(trackRecognition);

  printf("\n\n --11-  pathstype   =%d\n", pathstype);
  cv::Point2f point1, point2;

  int i = 1,j=1;
  float delta;
  int startrow = 0;
  int startrow2 = 0;


 
    cv::Mat image0= cv::Mat::zeros(800, 500, CV_8UC3);  

        // 设置线条颜色，这里使用白色
        cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
        cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);



  if (frame0.empty()) {
    cout << "Cannot open image!" << endl;
  }
  // --------------[02] 图像预处理
  // printf("----validRowsLeft =%d \n", trackRecognition.validRowsLeft);
  // printf("-----validRowsRight =%d \n", trackRecognition.validRowsRight);

 
  // int w = 2000, h = 2000;
  // float hpix = 340;
  // cv::Size size(w, h); // 新尺寸为300x200
  // cv::Mat image1,image0;      //
  // cv::resize(imageoriginal, image0, size);





/////////////////////////////////////////////--------------------------------
cv::Mat dstImg9 = cv::Mat::zeros(240, 320, CV_8UC3);
//Mat dstImg9; cv::Mat::zeros(800, 800, CV_8UC3);

 // ipm.homographyInv(frame0, dstImg9, cv::BORDER_CONSTANT);


    //



/////////////////////////////////////////------------------------------------------
////////////////////////////////-------------------------------

  Point2d startIpm;
  //
  vector<POINT> ipmpointsEdgeLeft, ipmpointsEdgeRight;
  for (int i = 0; i < trackRecognition.pointsEdgeRight.size(); i++)
  {
    startIpm = ipm.homography(Point2d(trackRecognition.pointsEdgeLeft[i].y, trackRecognition.pointsEdgeLeft[i].x));
    //

  // if (startIpm.y>0&&startIpm.x>3&&dstImg9.at<Vec3b>(startIpm.y, startIpm.x-2)[2]==0)// &&dstImg9.at<Vec3b>(startIpm.y, startIpm.x-2)[0]==0)
  //     startIpm.x = 0;

    // if (trackRecognition.pointsEdgeLeft[i].y < 2)
    //  

    ipmpointsEdgeLeft.push_back(POINT(startIpm.y, startIpm.x));



///////////////////////////--------pointsEdgeRight

    startIpm = ipm.homography(Point2d(trackRecognition.pointsEdgeRight[i].y, trackRecognition.pointsEdgeRight[i].x));
    //

Point2d startIipm = ipm.homographyInv(startIpm); // 反透视变换
    POINT startPoint = POINT(startIipm.y, startIipm.x);

 if (startPoint.y>317)
      startIpm.x =319;

//  if (startIpm.y>0&&startIpm.y<240&&startIpm.x>-1&&startIpm.x<317&&dstImg9.at<Vec3b>(startIpm.y, startIpm.x+3)[2]==0 &&dstImg9.at<Vec3b>(startIpm.y, startIpm.x+3)[0]==0)
//       startIpm.x =319;

    // if (trackRecognition.pointsEdgeRight[i].y > 317)
    //   startIpm.x = 319;

    ipmpointsEdgeRight.push_back(POINT(startIpm.y, startIpm.x));

    // int py = pointsEdge[i].y + offsetWidth;
    // printf("Right i=%d\t %d \t %d=\t%3.2f\t %3.2f \n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y,startIpm.y, startIpm.x);
    //
    // printf("Right i=%d\t %d \t %d=\t%3.2f\t %3.2f \n",i, trackRecognition.pointsEdgeRight[i].x, trackRecognition.pointsEdgeRight[i].y,startIpm.y, startIpm.x);
  }
//   for (int i =0; i <trackRecognition.pointsEdgeRight.size(); i++)
//             {

// printf("ipmpointsEdgei=%d- %d=\t%d=\t%d\t %d ---%d\n",i, ipmpointsEdgeLeft[i].x,ipmpointsEdgeLeft[i].y,  ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
//             }
//////////////////////-----分割环岛内圆
            int logo=0,s1 = 0, s2 = 0, cnt = 0;
            for (i = 0; i < ipmpointsEdgeRight.size(); i++)
            {

              if (ipmpointsEdgeRight[i].y > 318)
                cnt += 1;
              else
                cnt = 0;

              if (cnt > 40)
                break;
            }
            if (cnt > 40)
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

              for (j = s1; j < ipmpointsEdgeRight.size(); j++)
              {

                if (ipmpointsEdgeRight[j].y < 270)
                  cnt += 1;
                else
                  cnt = 0;

                if (cnt > 35)
                  break;
              }
            }

            if (cnt ==36)
            {

              for (i = j; i < ipmpointsEdgeRight.size(); i++)
              {
                if (ipmpointsEdgeRight[i].y > 317)
                {
                  s2 = i - 1;
                  break;
                }
                else if (ipmpointsEdgeRight[i].y<317&& i == ipmpointsEdgeRight.size() - 1)
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

if (logo>0&&s1>0&&s2-s1>30)
{

printf("Right s1=%d\t s2=%d \n",s1,s2);
            
    for (i =s1; i <s2; i++)
            {
  circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
                 5, colorLine1, 5); // 60行
   
          circle(dstImg9,
                 Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x), 5,
                 colorLine2, 2); // 60行
//printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, ipmpointsEdgeLeft[i].x,ipmpointsEdgeLeft[i].y,  ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y,ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
            }







///////////////////
  //////////////////////-----分割环岛内圆    
/////////////////////////---------------------------------------------


  // 生成一些二维离散点作为示例数据
    // Mat points(s2-s1+1, 1, CV_32FC2);  
     std::vector<cv::Point> points;
    for (int i =s1; i < s2; i++)
    { int x = static_cast<int>( ipmpointsEdgeRight[i].x);
    int y = static_cast<int>(ipmpointsEdgeRight[i].y);
       points.push_back(cv::Point(y, x));
    }
 // 生成点集
 
  

    // 检查点集是否至少包含5个点
    if (points.size() < 5) {
        std::cerr << "Need at least 5 points for fitEllipse" << std::endl;
        //return 0;
    }

// 使用fitEllipse拟合点集
    cv::RotatedRect ellipse = cv::fitEllipse(points);

    // 创建一个空白图像用于绘图
   // cv::Mat image = cv::Mat::zeros(800, 800, CV_8UC3);

    // 画出原始点
    // for (const auto& point : points) {
    //     cv::circle(image, point, 3, cv::Scalar(0, 255, 0), cv::FILLED);
    // }

    // 画出拟合的椭圆
   // cv::ellipse(image, ellipse, cv::Scalar(0, 0, 255), 2);

    // 显示图像
    //cv::imshow("Fitted Ellipse with Noise", image);


   


    // 绘制原始数据和拟合结果
//     Mat img(500, 500, CV_8UC3, Scalar(255, 255, 255));
//     for (int i = 0; i < points.rows; i++)
//     {
//         circle(dstImg9, Point(points.at<Vec2f>(i, 0)[0], points.at<Vec2f>(i, 0)[1]), 2, Scalar(0, 0, 255), FILLED);
//     }
// // 绘制拟合后的圆弧
//   
   cv::ellipse(dstImg9,ellipse, Scalar(0, 255, 0), 2);
  
     // imshow("Fitted Arc", img);

// 计算拟合误差
double error = 0.0;
cv::Point2f ellipseCenter = ellipse.center;
float avgRadius = (ellipse.size.width + ellipse.size.height) / 4;  // 长轴和短轴的平均值除以2
for (const auto& point : points) {
    double distance = cv::norm(cv::Point2f(point.x, point.y) - ellipseCenter);
    error += std::pow(distance - avgRadius, 2);
}

printf("Fitting error: %f\n", error);
printf("Ellipse center: (%f, %f)\n", ellipse.center.x, ellipse.center.y);
printf("Ellipse average radius: %f\n", avgRadius);
printf("Ellipse average radius width-height: %3.2f ::\t  %3.2f \n",ellipse.size.width*0.5,ellipse.size.height*0.5);
////////////////////////-------------------------------------
}
        for (int i = 0; i < trackRecognition.pointsEdgeRight.size(); i++) {
          circle(dstImg9, Point(ipmpointsEdgeLeft[i].y, ipmpointsEdgeLeft[i].x),
                 1, colorLine1, 5); // 60行
   
          circle(dstImg9,
                 Point(ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].x), 1,
                 colorLine2, 2); // 60行
        }
   cv::imshow("guan fangku--Transformed", dstImg9);



   cv::waitKey(20);
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


ipmpointsEdgeLeft.clear();


ipmpointsEdgeRight.clear();

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
};


///////////////////////////------------------240529


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
  int laneWidth = 35;
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




TrackRecognition ipm_pathstype_world(TrackRecognition &trackRecognition, Mat &frame0,int pathstype) //
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
    }
      // 这里可以添加绘图或其他处理逻辑

      printf("\n-- ipmpointsEdgeLeft.size()= %d \n", ipmpointsEdgeLeft.size());
      // for (i =0; i < pointsEdgeRightnew.size(); i++)
      // {
      //  printf("Right i=%d- %d=\t%d=\t%d\t %d ---%d\n",i, trackRecognition.pointsEdgeRight[i].x, pointsEdgeLeftnew[i].x, pointsEdgeLeftnew[i].y, pointsEdgeRightnew[i].y, pointsEdgeRightnew[i].y - pointsEdgeLeftnew[i].y);
      // }
  

    cv::Mat dstImg9 = cv::Mat::zeros(h, w, CV_8UC3);
    // 设置线条颜色，这里使用白色
    cv::Scalar colorLine1 = cv::Scalar(0, 255, 0);
    cv::Scalar colorLine2 = cv::Scalar(0, 255, 255);

    for (int i = 0; i < ipmpointsEdgeLeft.size(); i++)
    {
      circle(dstImg9, Point(ipmpointsEdgeLeft[i].y + w / 2, h - ipmpointsEdgeLeft[i].x),
             5, colorLine1, 5); // 60行

      // circle(dstImg9,
      //        Point(ipmpointsEdgeRight[i].y + w / 2, h - ipmpointsEdgeRight[i].x), 5,
      //        colorLine2, 2); // 60行
      // circle(dstImg9,
      //        Point((ipmpointsEdgeLeft[i].y + ipmpointsEdgeRight[i].y) * 0.5 + w / 2, h - ipmpointsEdgeRight[i].x), 8,
      //        Scalar(0, 0, 255), 2); // 60行
      // printf("---Right i=%d- %d=\t%d=\t%d\t %d ---%d\n", i, ipmpointsEdgeLeft[i].x, ipmpointsEdgeLeft[i].y, ipmpointsEdgeRight[i].x, ipmpointsEdgeRight[i].y, ipmpointsEdgeRight[i].y - ipmpointsEdgeLeft[i].y);
   
   
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

  vector<POINT> calCenterLine(const vector<POINT> &points)
  {

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

    if (ipm_pathstype == pathstype::pathLeftedge_only)//|| ipm_pathstype == pathstype::pathLeftxie
    {
      laneWidth = -laneWidth;
    }

    // 计算偏移量
    double offsetX = -laneWidth * cos(angle + M_PI / 2);
    double offsetY = -laneWidth * sin(angle + M_PI / 2);

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

     //savePointsToFile(centerEdgereal, "centerEdgereal.txt");

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
    if (ipm_pathstype == pathstype::pathRightedge_only )//|| ipm_pathstype == pathstype::pathRightxie
    {
      centerEdgereal = calCenterLine(ipmpointsEdgeRight);
    }

    else if (ipm_pathstype == pathstype::pathLeftedge_only )//|| ipm_pathstype == pathstype::pathLeftxie
    {
      centerEdgereal = calCenterLine(ipmpointsEdgeLeft);
    }

    else
    {
      centerEdgereal = calCenterLine(ipmpointsEdgeRight);
    }

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


///////////////////////////----------------------240529