<<<<<<< HEAD
#ifndef STRUCT_H
#define STRUCT_H

#include <opencv2/opencv.hpp>

struct Light {
    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f vertices[4];
    float length;         // 长度
    float width;          // 宽度
    float angle;          // 倾斜角度
    

    // 构造函数列表初始化
    Light() : length(0), width(0), angle(0), center(0, 0) {
        for (auto& v : vertices) {
            v = cv::Point2f(0, 0);
        }
    }

    // 额外添加一个带参构造函数，方便在 Armors.cpp 中直接创建
    explicit Light(cv::RotatedRect r) {
        rect = r;
        center = r.center;
        angle = r.angle;
        // 长边为 length，短边为 width
        length = std::max(r.size.width, r.size.height);
        width = std::min(r.size.width, r.size.height);
        r.points(vertices); 
=======
/*#ifndef STRUCT_H
#define STRUCT_H

// 1. 必须添加 memset 所需的头文件
#include <cstring>
// 2. OpenCV 头文件（修正为标准路径）
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

#include <vector>

//struct Light
{
    cv::RotatedRect rect;   // 灯条的旋转矩形
    float rat;             // 长宽比
    float AbsAngle;        // 灯条绝对角度
    cv::Point2f center;    // 灯条中心坐标
    cv::Point2f vertices[4]; // 灯条四个顶点坐标

    Light() 
    {
        // 现在 memset 可以正常使用，初始化 vertices 数组所有字节为 0
        //memset(vertices, 0, sizeof(vertices));
        
        // 替换 memset 为更安全的初始化
      std::fill(std::begin(vertices), std::end(vertices), cv::Point2f(0, 0));
    }
};//

// 装甲板结构体（适配YOLO+灯条检测）
struct Armors {
    //Light left;
    //Light right;

    cv::RotatedRect boundingRect;  // 装甲板外接旋转矩形
    cv::Point2f center;            // 装甲板中心坐标
    cv::Mat rvec, tvec;            // 旋转/平移向量

    std::vector<cv::Point2f> corners; // 装甲板四个角点（PnP解算用）
    //std::vector<Light> lights;     // 装甲板包含的灯条（保留灯条检测后填充）
    
    float confidence;             // 装甲板置信度
    int classId;             // YOLO类别（0=蓝，1=红）

    float distance;                // 相机到装甲板距离
    float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;        // 欧拉角
    
};

#endif
*/



#ifndef STRUCT_H
#define STRUCT_H


#include <opencv4/opencv2/opencv.hpp>

struct Light
{
    cv::RotatedRect rect;
    float rat;
    float AbsAngle;
    cv::Point2f center;
    cv::Point2f vertices[4];

    Light() 
    {
        memset(vertices, 0, sizeof(vertices));
>>>>>>> 33463e2bc191262bb0ef0e94a4143a5cba0005c8
    }
};

#endif
<<<<<<< HEAD
=======

>>>>>>> 33463e2bc191262bb0ef0e94a4143a5cba0005c8
