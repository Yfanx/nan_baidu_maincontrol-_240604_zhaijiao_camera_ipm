// #include "../include/common.hpp"

#include "../../include/common.hpp"
#include "../recognition/track_recognition.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// bool FindBridge(TrackRecognition &track, Mat &imageBinary)

bool FindBridge(TrackRecognition &track)
{

    //         TrackRecognition trackRecognition;            //0719  xiugai
    // //TrackRecognition track;         // 赛道识别
    // trackRecognition.rowCutUp=120;
    // trackRecognition.rowCutBottom=40;

    //       trackRecognition.trackRecognition(imageBinary);

    float avg_Ls = 0.0; // 左上1/5部分斜率平均值
    float avg_Le = 0.0; // 左下1/5部分斜率平均值

    float avg_Rs = 0.0;
    float avg_Re = 0.0;

    float kLs = 0.0;
    float kRs = 0.0;

    double x1, y1, x2, y2;

    float kLe = 0.0;
    float kRe = 0.0;
    printf("\n\n\n--qiao track.pointsEdgeLeft.size() =%d \t  track.stdevLeft=%3.2f  \t  track.stdevRight=%3.2f \n\n", track.pointsEdgeLeft.size(), track.stdevLeft, track.stdevRight);

    //                 for (int i = 0 ; i < track.pointsEdgeLeft.size() -2; i++)
    //                     {
    //    printf("qiao slope=%d=\t= %3.2f =\t%3.2f \t \n", i,track.pointsEdgeLeft[i+1].slope-track.pointsEdgeLeft[i].slope, track.pointsEdgeRight[i+1].slope-track.pointsEdgeRight[i].slope);

    //                       //  printf("qiao slope=%d=\t= %3.2f =\t%3.2f \t \n", i, track.pointsEdgeLeft[i].slope, track.pointsEdgeRight[i].slope);

    //                     }

    std::vector<double> slopeLeft = calculateSlopes_4(track.pointsEdgeLeft);
    std::vector<double> slopeRight = calculateSlopes_4(track.pointsEdgeRight);

    //    for (int i = 0 ; i < track.pointsEdgeLeft.size() -10; i++)
    //                     {
    //    //printf("qiao slope=%d=\t= %3.2f =\t%3.2f \t \n", i,slopeLeft[i+1]-slopeLeft[i], slopeRight[i+1]-slopeRight[i]);

    //         printf("qiao slope=%d=\t= %d =\t%d \t \n", i, track.widthBlock[i+8].y-track.widthBlock[i].y ,track.widthBlock[i].y);

    //                     }

    int srow = track.validRowsLeft > track.validRowsRight ? track.validRowsLeft : track.validRowsRight;

    if (track.pointsEdgeLeft.size() == 229 && track.stdevEdgeCal2(track.pointsEdgeLeft, track.validRowsLeft, track.pointsEdgeLeft.size() - 1) < 40 && track.stdevEdgeCal2(track.pointsEdgeRight, track.validRowsRight, track.pointsEdgeRight.size() - 1) < 40 && track.stdevEdgeCal2(track.pointsEdgeLeft, track.validRowsLeft, track.pointsEdgeLeft.size() - 1) > 10 && track.stdevEdgeCal2(track.pointsEdgeRight, track.validRowsRight, track.pointsEdgeRight.size() - 1) > 10 && track.validRowsLeft < 0.4 * track.pointsEdgeLeft.size() && track.validRowsRight < 0.4 * track.pointsEdgeRight.size())
    {
        /////////////////////////

        for (int i = srow; i < srow + 50; i++)
        {

            y1 = track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i + 3].y;
            x1 = track.pointsEdgeLeft[i].x - track.pointsEdgeLeft[i + 3].x;
            kLs += y1 / x1;
            //.push_back(k);

            y2 = track.pointsEdgeRight[i].y - track.pointsEdgeRight[i + 3].y;
            x2 = track.pointsEdgeRight[i].x - track.pointsEdgeRight[i + 3].x;
            kRs += y2 / x2;
            // slopeRight.push_back(k);
        }

        for (int i = track.pointsEdgeLeft.size() - 2; i > track.pointsEdgeLeft.size() - 48; i--)
        {

            y1 = track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i + 3].y;
            x1 = track.pointsEdgeLeft[i].x - track.pointsEdgeLeft[i + 3].x;
            kLe += y1 / x1;
            // slopeLeft.push_back(k);

            y2 = track.pointsEdgeRight[i].y - track.pointsEdgeRight[i + 3].y;
            x2 = track.pointsEdgeRight[i].x - track.pointsEdgeRight[i + 3].x;
            kRe += y2 / x2;
            // slopeRight.push_back(k);
        }
        // cout<<"第一步"<<endl;

        avg_Ls = kLs / 50;
        avg_Le = kLe / 50;

        avg_Rs = kRs / 50;
        avg_Re = kRe / 50;

        // cout<<"avg_Ls ="<<avg_Ls <<endl;
        // cout<<"avg_Rs ="<<avg_Rs  <<endl;
        // cout<<"avg_Lx ="<<avg_Lx <<endl;
        // cout<<"avg_Rx ="<<avg_Rx <<endl;

        // cout<<"第2步"<<endl;

        printf("\n\n\n track.pointsEdgeLeft.size() =%d \t  abs(avg_Ls - avg_Lx)=%3.5f  \t  abs(avg_Rs - avg_Rx)=%3.5f \n\n", track.pointsEdgeLeft.size(), abs(avg_Ls - avg_Le), abs(avg_Rs - avg_Re));

        // cout<<"abs(avg_Ls - avg_Lx)= "<<abs(avg_Ls - avg_Lx)<<endl;
        // cout<<"abs(avg_Rs - avg_Rx))= "<<abs(avg_Rs - avg_Rx)<<endl;

        // int cnum = 0;
        // cout<<"track.stdevLeft= "<<track.stdevLeft<<endl;
        // cout<<"track.stdevRight= "<<track.stdevRight<<endl;//&&track.stdevLeft > 9.5&&track.stdevRight>9.5
        if (abs(avg_Ls - avg_Le) < 2 && abs(avg_Rs - avg_Re) < 2 && abs(avg_Ls - avg_Le) > 0.2 && abs(avg_Rs - avg_Re) > 0.2)
        {
            cout << " 前方有桥" << endl;
            /*cnum++;
            /*if (cnum > 7)
            {

                return true;
            }*/
            return true;
        }
        else
            return false;
    }
    else
        return false;
    /*else
    {
        cnum = 0;
    }*/
}

/*
class Qiao
{
public:


    bool FindBridge(TrackRecognition &track)
    {
        float avg_Lx = 0.0; // 左下1/5部分斜率平均值
        float avg_Ls = 0.0; // 左上1/5部分斜率平均值
        float avg_Rx = 0.0;
        float avg_Rs = 0.0;

        for (int i = 0; i < track.pointsEdgeLeft.size() / 4 - 2; i++)
        {
            float kLx = 0.0;
            float kRx = 0.0;

            double y1 = track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i + 2].y;
            double x1 = track.pointsEdgeLeft[i].x - track.pointsEdgeLeft[i + 2].x;
            double kLx += y1 / x1;
            // slopeLeft.push_back(k);

            double y2 = track.pointsEdgeRight[i].y - track.pointsEdgeRight[i + 2].y;
            double x2 = track.pointsEdgeRight[i].x - track.pointsEdgeRight[i + 2].x;
            double kRx = y2 / x2;
            // slopeRight.push_back(k);
        }

        for (int i = track.pointsEdgeLeft.size() - 2; i > track.pointsEdgeLeft.size() * 0.75 - 2; i--)
        {
            float kLs = 0.0;
            float kRs = 0.0;

            double y1 = track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i + 2].y;
            double x1 = track.pointsEdgeLeft[i].x - track.pointsEdgeLeft[i + 2].x;
            double kLs += y1 / x2;
            // slopeLeft.push_back(k);

            double y2 = track.pointsEdgeRight[i].y - track.pointsEdgeRight[i + 2].y;
            double x2 = track.pointsEdgeRight[i].x - track.pointsEdgeRight[i + 2].x;
            double kRs += y1 / x2;
            // slopeRight.push_back(k);
        }

        avg_Ls = kLs / (track.pointsEdgeLeft.size() / 4);
        avg_Lx = kLx / (track.pointsEdgeLeft.size() / 4 - 2);

        avg_Rs = kRs / (track.pointsEdgeLeft.size() / 4);
        avg_Rx = kRx / (track.pointsEdgeLeft.size() / 4 - 2);

        int cnum = 0;
        if (abs(avg_Ls - avg_Lx) < 20 && abs(avg_Rs - avg_Rx) < 20 && track.stdevLeft > 20 &&
            track.stdevRight > 20 && track.stdevLeft < 80 && track.stdevRight < 80)
        {
            cnum++;
            if (cnum > 7)
            {
                cout " 前方有桥" << endl;
                return true;
            }
        }

        else
        {
            cnum = 0;
        }
    }





private:

};*/
