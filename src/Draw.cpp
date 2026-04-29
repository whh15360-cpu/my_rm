<<<<<<< HEAD
#include "Draw.hpp"
#include "Config.hpp"
#include <cstdio>
#include <cmath>


 // 绘制跟踪与预测可视化信息
void drawTrack(
    cv::Mat& img, 
    const Armors& detected_armor, 
    const cv::Point3f& predict_position, 
    double predict_yaw,
    const cv::Mat& last_valid_rvec
) {
    // 获取全局配置中心的数据
    auto& cfg = Config::getInstance();
    if (cfg.cameraMatrix.empty() || cfg.distCoeffs.empty()) return;

    // 固定几何参数 
    const float armorWidth  = 0.235f;
    const float armorHeight = 0.127f;
    const float DIST_TO_CENTER = 0.2f;  // 机器人中心到装甲板的距离，云台转轴中心到装甲板几何中心的垂直距离

    // 3D 点顺序与 solvePose 保持一致：左上、右上、右下、左下
    static const std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth/2, -armorHeight/2, 0.0f},
        { armorWidth/2, -armorHeight/2, 0.0f},
        { armorWidth/2,  armorHeight/2, 0.0f},
        {-armorWidth/2,  armorHeight/2, 0.0f}
    };

    // 姿态处理
    cv::Mat rvec;
    if (!detected_armor.rvec.empty()) {
        rvec = detected_armor.rvec.clone();
    } else if (!last_valid_rvec.empty()) {
        rvec = last_valid_rvec.clone();
    } else {
        // 若无有效旋转，根据预测 yaw 构造旋转向量
        cv::Mat rot_mat = (cv::Mat_<double>(3,3) <<
            cos(predict_yaw), -sin(predict_yaw), 0,
            sin(predict_yaw),  cos(predict_yaw), 0,
            0, 0, 1
        );
        cv::Rodrigues(rot_mat, rvec);
    }

    // 预测位置 tvec
    cv::Mat tvec = (cv::Mat_<double>(3,1) << 
        (double)predict_position.x,
        (double)predict_position.y,
        (double)predict_position.z
    );

    //  绘制检测到的装甲板（红色）
    if (!detected_armor.rvec.empty() && !detected_armor.tvec.empty()) {
        std::vector<cv::Point2f> pts;
        cv::projectPoints(armor_3d_points, detected_armor.rvec, detected_armor.tvec,
                          cfg.cameraMatrix, cfg.distCoeffs, pts);

        std::vector<cv::Point> int_pts;
        for (auto& p : pts) int_pts.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::polylines(img, int_pts, true, cv::Scalar(0, 0, 255), 2);
    }

    //  绘制预测的装甲板（绿色） 
    std::vector<cv::Point2f> pred_pts;
    cv::projectPoints(armor_3d_points, rvec, tvec, cfg.cameraMatrix, cfg.distCoeffs, pred_pts);
    
    std::vector<cv::Point> pred_int_pts;
    for (auto& p : pred_pts) pred_int_pts.emplace_back(cvRound(p.x), cvRound(p.y));
    cv::polylines(img, pred_int_pts, true, cv::Scalar(0, 255, 0), 2);

    // 绘制四面装甲板虚拟模型 
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    // 计算机器人中心：预测装甲板中心点往法线后方退 DIST_TO_CENTER
    cv::Mat robot_center = tvec + R.col(2) * DIST_TO_CENTER;

    std::vector<cv::Mat> dirs = { R.col(0), -R.col(2), -R.col(0), R.col(2) };
    std::vector<cv::Scalar> Colors = {
        {255,255,0}, {0,255,0}, {255,0,255}, {0,255,255}
    };

    for (int i = 0; i < 4; ++i) {
        cv::Mat new_tvec = robot_center + dirs[i] * DIST_TO_CENTER;
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(armor_3d_points, rvec, new_tvec, cfg.cameraMatrix, cfg.distCoeffs, img_pts);

        std::vector<cv::Point> int_pts;
        for (auto& p : img_pts) {
            int_pts.emplace_back(cvRound(std::max(0.f, std::min((float)img.cols-1, p.x))),
                                 cvRound(std::max(0.f, std::min((float)img.rows-1, p.y))));
        }
        cv::polylines(img, int_pts, true, Colors[i], 1);
    }

    // 信息文字
    char text[128];
    snprintf(text, sizeof(text), "Pos: %.2f, %.2f, %.2f", 
             predict_position.x, predict_position.y, predict_position.z);
    //cv::putText(img, text, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    cv::Point text_org(20, 60); // 左上角起始坐标
    
    cv::putText(img, text, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 4);

    cv::putText(img, text, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 0), 2); //出现黑色文字，显示dist距离
}
=======
// draw.cpp
/*#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "Struct.hpp"
#include "PoseSolve.hpp"

// 绘制识别结果（适配YOLO，保留核心可视化）
void ArmorsDetector::draw(cv::Mat &img, const std::vector<Armors> &armors) {
    if (img.empty()) return;

    // 无装甲板时提示
    if (armors.empty()) {
        cv::putText(img, "No Armors Detected", cv::Point(20, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        return;
    }

    // 遍历绘制每个装甲板
    for (size_t i = 0; i < armors.size(); i++) {
        const auto &armor = armors[i];

        // 1. 绘制装甲板外接矩形
        cv::Point2f armor_pts[4];
        armor.boundingRect.points(armor_pts);
        for (int k = 0; k < 4; k++) {
            cv::line(img, armor_pts[k], armor_pts[(k+1)%4], cv::Scalar(0, 255, 0), 2);
        }

        // 2. 绘制装甲板中心点
        cv::circle(img, armor.center, 3, cv::Scalar(0, 0, 255), -1);

        // 3. 格式化输出位姿信息（保留1位小数，更易读）
        std::stringstream tvec_ss;
        tvec_ss << std::fixed << std::setprecision(1);
        tvec_ss << "T: x=" << armor.tvec.at<double>(0)
                 << " y=" << armor.tvec.at<double>(1)
                 << " z=" << armor.tvec.at<double>(2)
                 << " dist=" << armor.distance << "m";

        // 4. 绘制文本
        cv::putText(img, tvec_ss.str(), cv::Point(10, 30 + (int)i*30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
    }
}*/
>>>>>>> 33463e2bc191262bb0ef0e94a4143a5cba0005c8
