#include "PoseSolve.hpp"
#include "Config.hpp"  
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>


ArmorsDetector::ArmorsDetector() 
{
    // 同步单例中的相机矩阵到成员变量，方便频繁调用
    auto& cfg = Config::getInstance();
    if (!cfg.cameraMatrix.empty()) {
        cameraMatrix_ = cfg.cameraMatrix.clone();
        distCoeffs_ = cfg.distCoeffs.clone();
    }
}


 // YOLO 对接接口
std::vector<Armors> ArmorsDetector::detect(const cv::Mat& frame, 
                                           const std::vector<cv::Rect>& yoloBoxes, 
                                           const std::vector<float>& confidences, 
                                           const std::vector<int>& classIds)
{
    std::vector<Armors> armors;
    if (frame.empty() || yoloBoxes.empty()) return armors;

    // 无需校验 config_ 指针，直接使用成员变量或单例
    for (size_t i = 0; i < yoloBoxes.size(); ++i) {
        const cv::Rect& box = yoloBoxes[i];
        
        // 基础信息填充
        Armors armor;
        armor.center = cv::Point2f(box.x + box.width/2.0f, box.y + box.height/2.0f);
        
        // 构造四角点：对应 3D 模型的 左上、右上、右下、左下
        armor.corners = {
            cv::Point2f(box.x, box.y),
            cv::Point2f(box.x + box.width, box.y),
            cv::Point2f(box.x + box.width, box.y + box.height),
            cv::Point2f(box.x, box.y + box.height)
        };

        if (solvePose(armor)) {
            armors.push_back(armor);
        }
    }
    return armors;
}

// 位姿解算核心

bool ArmorsDetector::solvePose(Armors& armor)
{
    //  定义 3D 模型点 (单位: 米) 
    // 顺序：左上(-,-)、右上(+,-)、右下(+,+)、左下(-,+) 对应图像坐标系 Y 向下
    static const std::vector<cv::Point3f> armor_3d = {
        {-0.1175f, -0.0635f, 0.0f},
        { 0.1175f, -0.0635f, 0.0f},
        { 0.1175f,  0.0635f, 0.0f},
        {-0.1175f,  0.0635f, 0.0f}
    };

    if (armor.corners.size() != 4) return false;

    //  PnP 求解
    cv::Mat rvec, tvec;
    auto& cfg = Config::getInstance(); // 实时拉取最新配置

    // 使用 SOLVEPNP_IPPE_SQUARE 专门针对矩形物体优化，比 EPNP 在装甲板场景更稳
    bool success = cv::solvePnP(armor_3d, armor.corners, 
                                cfg.cameraMatrix, cfg.distCoeffs, 
                                rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

    if (!success || !cv::checkRange(rvec) || !cv::checkRange(tvec)) return false;

    // 深度与距离校验
    double z = tvec.at<double>(2);
    if (z <= 0.1 || z > 15.0) return false; // 排除远距离解

    // 计算欧拉角
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    // 提取 Euler Angles (适应 RoboMaster 坐标系：Yaw 为水平转角)
    double yaw = atan2(R.at<double>(0, 2), R.at<double>(2, 2));
    double pitch = asin(-R.at<double>(1, 2));

    // 结果赋值
    armor.rvec = rvec.clone();
    armor.tvec = tvec.clone();
    armor.pos = cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    armor.distance = static_cast<float>(cv::norm(tvec));
    armor.yaw = static_cast<float>(yaw * 180.0 / CV_PI);
    armor.pitch = static_cast<float>(pitch * 180.0 / CV_PI);

    return true;
}

void ArmorsDetector::draw(cv::Mat &img, const std::vector<Armors> &armors)
{
    for (const auto& a : armors) {
        // 绘制检测框
        for (int i = 0; i < 4; ++i)
            cv::line(img, a.corners[i], a.corners[(i+1)%4], {0,255,0}, 2);

        // 打印坐标信息
        std::string info = cv::format("%.2fm, Y:%.1f", a.distance, a.yaw);
        cv::putText(img, info, a.corners[0], 1, 1.2, {0,255,255}, 2);
    }
}
