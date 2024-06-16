#pragma once

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../src/perspective_mapping.cpp"
#include <numeric> // 用于std::accumulate
//
using nlohmann::json;     //PC上跑时，需要,EDGEBOARD时，需注释掉
using namespace std;
using namespace cv;

#define COLSIMAGE 320       // 图像的列数
#define ROWSIMAGE 240       // 图像的行数
#define COLSIMAGEIPM 320    // IPM图像的列数
#define ROWSIMAGEIPM 400    // IPM图像的行数
#define SERVO_PWM_MAX_R 970 // 舵机PWM最大值（左）1840   -----方向左边最大值
#define PWMSERVOMID 780     // 舵机PWM中值 1520
#define SERVO_PWM_MAX_L 590 // 舵机PWM最小值（右）1200

 enum pathstype {   
None = 0,
pathStraight, // 直入十字

pathLeftedge_only,  // 左斜入十字
pathRightedge_only, // 右斜入十字
 
// pathLeftxie, 
// pathRightxie, // 右斜入十字
 
pathLeftwan,  // 左斜入十字
pathRightwan, // 右斜入十字
 
};


#define LABEL_CONE "cone"           // AI标签：锥桶
#define LABEL_GRANARY "granary"     // AI标签：谷仓
#define LABEL_BRIDGE "bridge"       // AI标签：桥
#define LABEL_TRACTOR "tractor"     // AI标签：拖拉机
#define LABEL_CORN "corn"           // AI标签：玉米
#define LABEL_PIG "pig"             // AI标签：猪
#define LABEL_CROSSWALK "crosswalk" // AI标签：斑马线
#define LABEL_BUMP "bump"           // AI标签：减速带

bool printAiEnable = false;
PerspectiveMapping ipm; // 逆透视变换公共类

// 鼠标回调函数实现
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) { // 检测到左键点击
        cv::Mat& img = *reinterpret_cast<cv::Mat*>(userdata);

        // 在图片上标记坐标（x, y）
        std::string coordText = "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
        cv::putText(img, coordText, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // 刷新显示，更新窗口以显示新的图像
        cv::imshow("Image", img);
    }
}




struct POINT
{
    int x = 0;
    int y = 0;
    float slope = 0.0f;

    POINT(){};
    POINT(int x, int y) : x(x), y(y){};
};


double vecmax(vector<POINT> vec,int start)
{
   
    // 使用lambda表达式作为自定义比较函数
    auto max_it = std::max_element(vec.begin()+start, vec.end(),
                                   [](const POINT& a, const POINT& b) {
                                       return a.y < b.y;
                                   });

    // 输出x值最大的Point对象
    if (max_it != vec.end()) {
        std::cout << "The Point with the largest x value is: "
                  << "(" << max_it->y << ", " << max_it->x << ")" << std::endl;
    } else {
        std::cout << "The vector is empty" << std::endl;
    }

    return max_it->y ;
}
double vecmin(vector<POINT> vec,int start)
{
   
    // 使用lambda表达式作为自定义比较函数
    auto min_it = std::min_element(vec.begin()+start, vec.end(),
                                   [](const POINT& a, const POINT& b) {
                                       return a.y >b.y;
                                   });

    // 输出x值最大的Point对象
    if (min_it != vec.end()) {
        std::cout << "The Point with the least x value is: "
                  << "(" << min_it->y << ", " << min_it->x << ")" << std::endl;
    } else {
        std::cout << "The vector is empty" << std::endl;
    }

    return min_it->y ;
}
/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image)
{
    // 存图
    string name = ".png";
    static int counter = 0;
    counter++;
    string img_path = "../res/samples/train/";
    name = img_path + to_string(counter) + ".jpg";
    imwrite(name, image);
}

void savePictureo(Mat &image)
{
    // 存图
    string name = ".png";
    static int counter = 0;
    counter++;
    string img_path = "../res/samples/traino/";
    name = img_path + to_string(counter) + ".jpg";
    imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}


double averagepoint(vector<POINT> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    return aver ;
}
double averagepoint_valid(vector<POINT> vec,int s)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (int i = s; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / (vec.size()-s); // 集合平均值

    return aver ;
}
/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}


std::vector<double> calculateSlopes(const vector<POINT>points) {
    std::vector<double> slopes;

    if (points.size()>10)
    {
  for (size_t i =1; i < points.size(); i++) {
        double dx = points[i].x - points[i-1].x;
        double dy = points[i].y - points[i-1].y;
        
        //保证dx！=0 
        if(dx == 0){
            dx = 0.001;
        }

        if (dx != 0) {
            double slope = dy / dx;
            if (!std::isinf(slope) && !std::isnan(slope)) {
                slopes.push_back(slope);
            }
        }
    }

    }
  
    return slopes;
}




std::vector<double> calculateSlopes_4(const vector<POINT>points) {
    std::vector<double> slopes;

    if (points.size()>10)
    {
  for (size_t i =0; i < points.size()-5; i++) {
        double dx = points[i+4].x - points[i].x;
        double dy = points[i+4].y - points[i].y;
        
        //保证dx！=0 
        if(dx == 0){
            dx = 0.0001;
        }

        if (dx != 0) {
            double slope = dy / dx;
            if (!std::isinf(slope) && !std::isnan(slope)) {
                slopes.push_back(slope);
            }
        }
    }

    }
  
    return slopes;
}




/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<POINT> vec)
{
    if (vec.size() < 1)
        return 0;

    
    double aver = averagepoint(vec); // 集合平均值

    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}


double sigma_valid(vector<POINT> vec,int s)
{
    if (vec.size() <5)
        return 0;

 
    double aver = averagepoint_valid(vec,s); // 集合平均值

    double sigma = 0;
    for (int i =s; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)(vec.size()-s);
    return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

double deg2rad(double degrees)
{
    return degrees * 3.14159 / 180.0;
}

double rad2deg(double degrees)
{
    return degrees * 180.0/3.14159  ;
}
/**
 * @brief 直线拟合算斜率
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
 double centerline_xielv(vector<POINT> centerEdge)
{
// Fit a line to the points

if(centerEdge.size()>5)
{std::vector<cv::Point2f> points;
 for (auto p : centerEdge)
        {
        points.emplace_back(cv::Point2f(p.x, p.y));
        }
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);

    // Calculate the angle of the line
    double vx = line[0], vy = line[1];
    double a = std::atan2(vy, vx) * 180.0 / CV_PI;

return a;}
else
return 0;

}
/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
vector<POINT> Bezier(double dt, vector<POINT> input)
{
    vector<POINT> output;

    double t = 0;
    while (t <= 1)
    {
        POINT p;
        double x_sum = 0.0;
        double y_sum = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n)
        {
            double k =
                factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            x_sum += k * input[i].x;
            y_sum += k * input[i].y;
            i++;
        }
        p.x = x_sum;
        p.y = y_sum;
        output.push_back(p);
        t += dt;
    }
    return output;
}
auto formatInt2String( int val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}

auto formatDoble2String(double val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(POINT a, POINT b, POINT p)
{
    int d = 0; // 距离

    double ab_distance =
        sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    double ap_distance =
        sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
    double bp_distance =
        sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

    double half = (ab_distance + ap_distance + bp_distance) / 2;
    double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

    return (2 * area / ab_distance);
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

class IcarShow
{
private:
    bool enable = false; // 显示窗口使能
    int sizeWindow = 1;  // 窗口数量
    cv::Mat imgShow;     // 窗口图像
public:
    /**
     * @brief 显示窗口初始化
     *
     * @param size 窗口数量(1~7)
     */
    void init(int size)
    {
        if (size <= 0 || size > 7)
            return;

        cv::namedWindow("icar", WINDOW_NORMAL);                // 图像名称
        cv::resizeWindow("icar", COLSIMAGE * size, ROWSIMAGE); // 分辨率

        imgShow = cv::Mat::zeros(ROWSIMAGE, COLSIMAGE * size, CV_8UC3);
        enable = true;
        sizeWindow = size;
    }

    /**
     * @brief 设置新窗口属性
     *
     * @param index 窗口序号
     * @param img 显示图像
     * @param name 窗口名称
     */
    void setNewWindow(int index, Mat img, string name)
    {
        // 数据溢出保护
        if (!enable || index < 0 || index >= sizeWindow)
            return;
        if (img.cols <= 0 || img.rows <= 0)
            return;

        Mat imgDraw = img.clone();

        if (imgDraw.type() == CV_8UC1) // 非RGB类型的图像
            cvtColor(imgDraw, imgDraw, cv::COLOR_GRAY2BGR);

        // 图像缩放
        if (imgDraw.cols != COLSIMAGE || imgDraw.rows != ROWSIMAGE)
        {
            float fx = COLSIMAGE / imgDraw.cols;
            float fy = ROWSIMAGE / imgDraw.rows;
            if (fx <= fy)
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fx, fx);
            else
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fy, fy);
        }

        // 限制图片标题长度
        string text = "[" + to_string(index + 1) + "] ";
        if (name.length() > 15)
            text = text + name.substr(0, 15);
        else
            text = text + name;

        putText(imgDraw, text, Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 0.5, CV_AA);

        Rect placeImg = cvRect(COLSIMAGE * index, 0, COLSIMAGE, ROWSIMAGE);
        imgDraw.copyTo(imgShow(placeImg));
    }

    /**
     * @brief 融合后的图像显示
     *
     */
    void display()
    {
        if (enable)
            imshow("icar", imgShow);
    }
};
///////////////////////



int enlageshow(Mat src) {
    // 加载图像
  
    // 检查图像是否成功加载
    if(src.empty()) {
        std::cerr << "Error: Loading image" << std::endl;
        return -1;
    }

    // 创建一个空矩阵用来存放放大后的图像
    cv::Mat dst;

    // 放大倍数
    double scale =3.5;

    // 使用双线性插值放大图像
    cv::resize(src, dst, cv::Size(), scale, scale, cv::INTER_LINEAR);

    // 创建窗口
   // cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Enlarged Image", cv::WINDOW_AUTOSIZE);

    // 显示图像
    //cv::imshow("Original Image", src);
    cv::imshow("Enlarged Image", dst);

    // 等待按键
    cv::waitKey(20);

    return 0;
}



//////////

void savePointsToFile(const std::vector<POINT>& points, const std::string& filePath) {
    std::ofstream outFile(filePath);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file at " << filePath << std::endl;
        return;
    }

    for (const auto& point : points) {
        //outFile << "x: " << point.x << ", y: " << point.y << ", slope: " << point.slope << "\n";
        outFile << point.x << "\t" << point.y << "\n";
    }

    outFile.close();
    std::cout << "Data written successfully to " << filePath << std::endl;
}




////////////////
//********************
void write_pwm(const string& file_path, int data, ofstream& fout)
{
    fout << "   pwm:   " << data;
}
////////////////////////////////////////////////
void write_error(int data, ofstream& fout)
{
    fout << "   error:   " << data << endl;
}
////////////////////////////////////////////////





////////////////////////////////////////////////
