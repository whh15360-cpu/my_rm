#include "DrawTrack.hpp"
#include <cstdio>
#include <cmath>
#include <opencv2/opencv.hpp>

void drawTrack(cv::Mat& img, 
            const Armors& detected_armor, 
            const cv::Point3f& predict_position, 
            double predict_yaw,
            const cv::Mat& last_valid_rvec,
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs)
{
    const float armorWidth  = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2, -armorHeight/2, 0.0f},
        {-armorWidth/2, -armorHeight/2, 0.0f}
    };

    //=====================================
    // 取当前一块装甲板的位姿
    //=====================================
    cv::Mat rvec, tvec;
    if (!detected_armor.rvec.empty() && !detected_armor.tvec.empty()) {
        rvec = detected_armor.rvec.clone();
        tvec = detected_armor.tvec.clone();
    } else if (!last_valid_rvec.empty()) {
        rvec = last_valid_rvec.clone();
        tvec = (cv::Mat_<double>(3,1) << predict_position.x, predict_position.y, predict_position.z);
    } else {
        rvec = cv::Mat::zeros(3, 1, CV_64F);
        tvec = (cv::Mat_<double>(3,1) << 0, 0, 0);
    }

    if (tvec.at<double>(2) < 0.5)
        tvec.at<double>(2) = 0.5;

    //=====================================
    // 核心：单装甲板反推机器人中心
    // 装甲板到中心 = 0.5m
    //=====================================
    const float DIST_TO_CENTER = 0.2f;

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 机器人中心 = 装甲板位置 - 法线方向 * 0.5m
    cv::Mat robot_center = tvec + R.col(2) * DIST_TO_CENTER;

    // 四个方向：0° 90° 180° 270°
    std::vector<double> yaw_rots = {
        0, CV_PI/2, CV_PI, -CV_PI/2
    };

    std::vector<cv::Scalar> colors = {
        {0,255,0},    // 原装甲板 绿
        {255,255,0},  // 推算1 黄
        {0,255,255},  // 推算2 青
        {255,0,255}   // 推算3 紫
    };

    //=====================================
    // 生成四块装甲板（严格符合物理逻辑 + 全框对）
    //=====================================
    // 四个方向：前、右、后、左（沿机器人自身坐标系）
    std::vector<cv::Mat> dirs = {
        R.col(0),        // 前：法线方向
        -R.col(2),        // 右：装甲板水平右向
        -R.col(0),       // 后：法线反方向
        R.col(2)        // 左：装甲板水平左向
    };
    std::vector<cv::Scalar> Colors = {{255,255,0}, {0,255,0}, {255,0,255}, {0,255,255}};

    for (int i = 0; i < 4; ++i)
    {
        // // 姿态：正面装甲板姿态不变，左右/背面绕Z轴旋转
        // double yaw = i * CV_PI/2;
        // cv::Mat Rz = (cv::Mat_<double>(3,3) <<
        //     cos(yaw), -sin(yaw), 0,
        //     sin(yaw),  cos(yaw), 0,
        //     0, 0, 1);
        // cv::Mat new_R = R * Rz;
        // cv::Mat new_rvec;
        // cv::Rodrigues(new_R, new_rvec);

        // 姿态：不做旋转偏移
        cv::Mat new_rvec = rvec.clone();

        // 位置：从机器人中心沿「机器人自身方向」偏移 0.5m
        cv::Mat new_tvec = robot_center + dirs[i] * DIST_TO_CENTER;

        // 投影并绘制（边界裁剪避免框出画面）
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(armor_3d_points, new_rvec, new_tvec, camera_matrix, dist_coeffs, img_pts);
        std::vector<cv::Point> int_pts;
        for (auto& p : img_pts) {
            p.x = std::max(0.f, std::min((float)img.cols-1, p.x));
            p.y = std::max(0.f, std::min((float)img.rows-1, p.y));
            int_pts.emplace_back(cvRound(p.x), cvRound(p.y));
        }
        if (int_pts.size() == 4)
            cv::polylines(img, int_pts, true, Colors[i], 2);
    }

    // 信息显示
    char text1[100];
    snprintf(text1, sizeof(text1), "Predict: (%.2f, %.2f, %.2f)", predict_position.x, predict_position.y, predict_position.z);
    cv::putText(img, text1, {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,255,255}, 2);
}
