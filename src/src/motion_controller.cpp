#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *                      (c) Copyright 2022; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-FZ3B),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file motion_controller.cpp
 * @author Leo ()
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2022-02-22
 * @note PD控制器要求稳定的控制周期：~40ms
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter_cal.cpp"

using namespace std;
//using nlohmann::json;

class MotionController
{
private:
    int counterShift = 0; // 变速计数器
public:
    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float speedLow = 0.2;                           // 智能车最低速
        float speedHigh = 0.3;                          // 智能车最高速
        float speedDown = 0.1;                          // 特殊区域降速速度
        float speedBridge = 0.3;                        // 坡道（桥）行驶速度
        float speedring = 0.2;                      // 慢行区行驶速度
        float speedGarage = 0.2;                        // 出入车库速度
        float runP1 = 0.9;                              // 一阶比例系数：直线控制量
        float runP2 = 0.018;                            // 二阶比例系数：弯道控制量
        float runP3 = 0.0;                              // 三阶比例系数：弯道控制量
        float turnP = 3.5;                              // 一阶比例系数：转弯控制量
        float turnD = 3.5;                              // 一阶微分系数：转弯控制量
        bool  debug = false;                             // 调试模式使能
        bool  saveImage = false;                         // 存图使能
        uint16_t rowCutUp = 10;                         // 图像顶部切行
        uint16_t rowCutBottom = 10;                     // 图像顶部切行
        uint16_t validRowscenter=20;
        float disGarageEntry = 0.7;                     // 车库入库距离(斑马线Image占比)
        bool GarageEnable = true;                       // 出入库使能
        bool BridgeEnable = true;                       // 坡道使能
        bool FreezoneEnable = true;                     // 泛行区使能
        bool RingEnable = true;                         // 环岛使能
        bool CrossEnable = true;                        // 十字使能
        bool GranaryEnable = true;                      // 粮仓使能
        bool DepotEnable = true;                        // 修车厂使能
        bool FarmlandEnable = true;                     // 农田使能
        bool SlowzoneEnable = true;                     // 慢行区使能
        uint16_t circles = 2;                           // 智能车运行圈数
        string pathVideo = "../res/samples/sample.mp4"; // 视频路径
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedDown, speedBridge,speedring, speedGarage,
                                       runP1, runP2, runP3, turnP, turnD, debug, saveImage, rowCutUp, rowCutBottom,validRowscenter,disGarageEntry,
                                       GarageEnable, BridgeEnable, FreezoneEnable, RingEnable, CrossEnable, GranaryEnable, DepotEnable, FarmlandEnable, SlowzoneEnable, circles, pathVideo); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    float motorSpeed = 1.0;          // 发送给电机的速度
    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
   

   

    /////////   tpl          ///////////////////
    /*void pdController(int controlCenter, double sigmaCenter)
    {
        float error = controlCenter - 160; // 图像控制中心转换偏差    ---左  负数
        save_data_error(int(error));       //** 保存数据error 文件在 build 下error 文件为多次的保存当前在最下面
        static int errorLast = 0;          // 记录前一次的偏差

        if (abs(error - errorLast) > 45)
        {
            params.turnP = 0.7+abs(error)*0.1;
            if(params.turnP>1.5)
            params.turnP=0.75;
        }
        else
        {
            params.turnP = abs(error) * params.runP2 + params.runP1;
        }

        int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnP*1.2;
        errorLast = error;

        servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff);     // PWM转换
   
    }*/

    /////////        
    
    
      void pdController(ControlCenterCal &controlCenterCal)
    {
int controlCenter=controlCenterCal.ipm_controlCenter;

double jiaodu=controlCenterCal.jiaodu;

         
          // 要写入的文件路径
        float error=0-controlCenter; // 图像控制中心转换偏差    ---左  负数 ----右  正数
                                        
        static int errorLast = 0; // 记录前一次的偏差
        if (abs(error - errorLast) > 32)
        {
            error = error>errorLast? errorLast+32:errorLast-32;
        }
        /*if (sigmaCenter < 100)

            params.turnP = abs(error) * params.runP2 + params.runP1;
        else
            params.turnP = abs(error) * (params.runP2 * 2.5) + params.runP1;
*/
        //params.turnP =abs(jiaodu) * params.runP2 + params.runP1;

        params.turnP =params.runP1;
        int pwmDiff = (error * params.turnP) + (error-errorLast)*params.turnD;
        errorLast = error;

        servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
       //等待按键

        // save_data_pwm(int(servoPwm));          //**保存pwm数据
        


    }
    
    
    
            ///////////////////

    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedController(bool enable, bool slowDown, ControlCenterCal control)
    {
        // 控制率
        uint8_t controlLow = 0;   // 速度控制下限
        uint8_t controlMid = 0.5; // 控制率
        uint8_t controlHigh = 1;  // 速度控制上限

        if (slowDown)
        {
            counterShift = controlLow;
            motorSpeed = params.speedDown;
        }
        else if (enable) // 加速使能
        {
            if (control.centerEdge.size() < 10)
            {
                motorSpeed = params.speedLow;
                counterShift = controlLow;
                return;
            }
            if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2)
            {
                motorSpeed = params.speedLow;
                counterShift = controlLow;
                return;
            }
            if (abs(control.sigmaCenter) < 100.0)
            {
                counterShift++;
                if (counterShift > controlHigh)
                    counterShift = controlHigh;
            }
            else
            {
                counterShift--;
                if (counterShift < controlLow)
                    counterShift = controlLow;
            }
            if (counterShift > controlMid)
                motorSpeed = params.speedHigh;
            else
                motorSpeed = params.speedLow;
        }
        else
        {
            counterShift = controlLow;
            motorSpeed = params.speedLow;
        }
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams()
    {
        string jsonPath = "../src/config/motion.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good())
        {
            std::cout << "Error: Params file path:[" << jsonPath
                      << "] not find .\n";
            exit(-1);
        }
nlohmann::json js_value;
    config_is >> js_value;
        //json js_value;
       // config_is >> js_value;

        try
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        motorSpeed = params.speedLow;
        cout << "--- runP1:" << params.runP1 << " | runP2:" << params.runP2 << " | runP3:" << params.runP3 << endl;
        cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
        cout << "--- speedLow:" << params.speedLow << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
    }
};