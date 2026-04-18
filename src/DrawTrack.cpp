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
    // 定义装甲板三维参考点
    const float armorWidth = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points;
    armor_3d_points.emplace_back(-armorWidth/2, armorHeight/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth/2, armorHeight/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth/2, -armorHeight/2, 0.0f);
    armor_3d_points.emplace_back(-armorWidth/2, -armorHeight/2, 0.0f);
    
    // ========== 绘制检测轮廓（红色）==========
    if (!detected_armor.rvec.empty() && !detected_armor.tvec.empty()) {
        std::vector<cv::Point2f> detected_points;
        cv::projectPoints(armor_3d_points, detected_armor.rvec, detected_armor.tvec, 
                         camera_matrix, dist_coeffs, detected_points);
        
        std::vector<cv::Point> detected_points_int;
        for (const auto& p : detected_points) {
            detected_points_int.emplace_back(cvRound(p.x), cvRound(p.y));
        }
        
        if (detected_points_int.size() == 4) {
            cv::polylines(img, detected_points_int, true, cv::Scalar(0, 0, 255), 2);
        }
    }
    
    // ========== 绘制预测轮廓（绿色）==========
    
    // 选择 rvec（优先用检测的姿态）
    cv::Mat rvec;
    if (!detected_armor.rvec.empty()) {
        rvec = detected_armor.rvec.clone();
    } else if (!last_valid_rvec.empty()) {
        rvec = last_valid_rvec.clone();
    } else {
        // 无任何历史：用预测的 yaw 构造 rvec
        cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64F);
        rot_mat.at<double>(0, 0) = cos(predict_yaw);
        rot_mat.at<double>(0, 1) = -sin(predict_yaw);
        rot_mat.at<double>(1, 0) = sin(predict_yaw);
        rot_mat.at<double>(1, 1) = cos(predict_yaw);
        rot_mat.at<double>(2, 2) = 1.0;
        cv::Rodrigues(rot_mat, rvec);
    }
    
    // 强制使用预测的 tvec
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 
                   static_cast<double>(predict_position.x),
                   static_cast<double>(predict_position.y),
                   static_cast<double>(predict_position.z));
    
    // 兜底：限制深度
    if (tvec.at<double>(2) < 0.5) 
    {
        tvec.at<double>(2) = 0.5;
    }
    
    // 投影预测的3D点到2D
    std::vector<cv::Point2f> predict_image_points;
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix, dist_coeffs, predict_image_points);
    
    std::vector<cv::Point> predict_image_points_int;
    for (const auto& p : predict_image_points) {
        predict_image_points_int.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    
    if (predict_image_points_int.size() == 4) {
        cv::polylines(img, predict_image_points_int, true, cv::Scalar(0, 255, 0), 2);
    }
    
    // 绘制预测位置中心点（青色十字）
    std::vector<cv::Point2f> center_projected_vec;
    std::vector<cv::Point3f> center_3d = {cv::Point3f(0, 0, 0)};
    cv::projectPoints(center_3d, rvec, tvec, camera_matrix, dist_coeffs, center_projected_vec);

    if (!center_projected_vec.empty()) {
        cv::Point center_int(cvRound(center_projected_vec[0].x), cvRound(center_projected_vec[0].y));
        cv::drawMarker(img, center_int, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
    }
    
    // 显示滤波后坐标
    char text1[100];
    snprintf(text1, sizeof(text1), "Predict: (%.2f, %.2f, %.2f)", 
             predict_position.x, predict_position.y, predict_position.z);
    cv::putText(img, text1, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 2);
    
    char text2[100];
    snprintf(text2, sizeof(text2), "Predict Yaw: %.2f deg", predict_yaw);
    cv::putText(img, text2, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 2);
}
