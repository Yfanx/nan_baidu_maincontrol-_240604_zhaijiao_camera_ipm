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
};