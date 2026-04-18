/*#ifndef POSE_SOLVE_H
#define POSE_SOLVE_H

// 核心依赖
#include <opencv2/opencv.hpp>
#include <vector>
#include "Struct.hpp"
//#include "Config.hpp"

// 装甲板结构体（包含位姿解算结果和YOLO信息）
/*struct Armors
{
    cv::Point2f center;          // 装甲板中心点
    std::vector<cv::Point2f> corners; // 装甲板四角点（位姿解算核心）
    cv::RotatedRect boundingRect; // 外接旋转矩形（绘制/匹配用）

    // 位姿解算结果
    cv::Mat rvec;                // 旋转向量
    cv::Mat tvec;                // 平移向量
    float distance = 0.0f;       // 相机到装甲板距离（米）
    float yaw = 0.0f;            // 偏航角（度）
    float pitch = 0.0f;          // 俯仰角（度）
    float roll = 0.0f;           // 翻滚角（度）

    // YOLO相关信息
    float confidence;     // YOLO检测置信度
    int classId;             // YOLO类别（0=蓝，1=红）

    // 构造函数：初始化矩阵避免空指针
    Armors() : rvec(cv::Mat::zeros(3, 1, CV_64F)), tvec(cv::Mat::zeros(3, 1, CV_64F)) {}
};

class ArmorsDetector
{
public:
    // 构造函数：初始化相机内参和畸变参数
    ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    ArmorsDetector() = default;

    // 接口1：仅YOLO检测（预留，可自行实现）
    std::vector<Armors> detectYoloArmors(const cv::Mat& frame);
    
    // 接口2：对外统一接口（兼容旧代码）
    std::vector<Armors> detect(const cv::Mat frame);


    // 接口3：YOLO对接核心接口（新增，适配卡尔曼滤波）
    std::vector<Armors> detect(const cv::Mat& frame, 
                               const std::vector<cv::Rect>& yoloBoxes, 
                               const std::vector<float>& confidences, 
                               const std::vector<int>& classIds);

    // 友元函数：外部绘制接口（保留原有声明）
    friend void drawFriend(ArmorsDetector& detector, cv::Mat& img, const std::vector<Armors> &armors);

private:

    bool solveArmorPose(Armors &armor);

    // 绘制结果（可视化调试）
    void draw(cv::Mat &img, const std::vector<Armors> &armors);

    // 相机内参和畸变参数（位姿解算）
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    // 配置相关（对接Config.hpp）
    //Config* config_ptr_ = nullptr;
    //const AppConfig* config_ = nullptr;

    // 装甲板固定物理尺寸（单位：米，统一单位避免换算错误）
    static constexpr float armorWidth_ = 0.141f;   // 141mm → 0.141m
    static constexpr float armorHeight_ = 0.125f;  // 125mm → 0.125m
};

// 友元函数声明（保留你的原有结构）
void drawFriend(ArmorsDetector& detector, cv::Mat& img, const std::vector<Armors> &armors);

#endif // POSE_SOLVE_H
*/


#ifndef POSE_SLOVE_H
#define POSE_SLOVE_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "Struct.hpp"
#include "Congfig.hpp"

struct Armors
{
    Light left;
    Light right;
    cv::RotatedRect boundingRect;
    cv::Point2f center;
    std::vector<cv::Point2f> corners;
    cv::Mat rvec;
    cv::Mat tvec;
    float distance = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f; 
    float roll = 0.0f;
};

class ArmorsDetector
{
public:
// 构造函数，初始化相机内参和畸变参数
    ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
// 核心接口：输入图像中的装甲板，输出其位姿信息
    std::vector<Armors> detect(const cv::Mat frame);
// 友元
    friend void drawFriend(ArmorsDetector& detector, cv::Mat& img, const std::vector<Light> &lights, const std::vector<Armors> &armors);
private:
    // 图像预处理
    cv::Mat preprocessImage(const cv::Mat &frame);
    // 灯条检测
    std::vector<Light> detectLights(const cv::Mat &mask);
    // 装甲板配对
    std::vector<Armors> matchArmors(const std::vector<Light> &lights);
    // 位姿估计
    bool solveArmorPose(Armors &armor);
    // 绘制
    void draw(cv::Mat &img, const std::vector<Light> &lights, const std::vector<Armors> &armors);

    // 相机内参和畸变参数
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    // 配置
    Config* config_ptr_;
    const AppConfig& config_;
    // 装甲板固定物理尺寸（单位：米）
    const float armorWidth_ = 0.141f;
    const float armorHeight_ = 0.125f;
};

#endif // POSE_SLOVE_H
