#include "PoseSolve.hpp"
#include "Config.hpp" 
#include <iostream>


// 构造函数：由于 Config 是单例，这里只需初始化日志或基础状态
ArmorsDetector::ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
    : cameraMatrix_(cameraMatrix.clone()), distCoeffs_(distCoeffs.clone()) {
}

std::vector<Armors> ArmorsDetector::detect(const cv::Mat& frame, 
                                           const std::vector<cv::Rect>& yoloBoxes, 
                                           const std::vector<float>& confidences, 
                                           const std::vector<int>& classIds) {
    std::vector<Armors> armors;
    if (frame.empty()) return armors;

    for (size_t i = 0; i < yoloBoxes.size(); ++i) {
        // 边界检查与处理
        cv::Rect roi = yoloBoxes[i] & cv::Rect(0, 0, frame.cols, frame.rows);
        if (roi.width <= 0 || roi.height <= 0) continue;

        cv::Mat roiImg = frame(roi);
        cv::Mat gray, binary;
        cv::cvtColor(roiImg, gray, cv::COLOR_BGR2GRAY);
        // 使用类成员 light_min_area 等进行阈值处理
        cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        std::vector<Light> lights;
        for (const auto& ct : contours) {
            if (ct.size() < 5) continue;
            cv::RotatedRect r = cv::minAreaRect(ct);
            Light l(r);
            
            // 使用类内定义的硬编码阈值
            if (l.rect.size.area() > light_min_area && std::abs(l.rect.angle) < light_max_angle) {
                // 将 ROI 局部坐标还原为全图坐标
                l.center += cv::Point2f(roi.x, roi.y);
                for (int k = 0; k < 4; k++) l.vertices[k] += cv::Point2f(roi.x, roi.y);
                lights.push_back(l);
            }
        }

        // 2. 装甲板匹配逻辑 
        for (size_t j = 0; j < lights.size(); j++) {
            for (size_t k = j + 1; k < lights.size(); k++) {
                const auto& l1 = lights[j];
                const auto& l2 = lights[k];
                
                if (std::abs(l1.angle - l2.angle) < light_max_angle_diff) {
                    Armors armor;
                    // 确保左右顺序
                    const auto& leftBar = (l1.center.x < l2.center.x) ? l1 : l2;
                    const auto& rightBar = (l1.center.x < l2.center.x) ? l2 : l1;

                    // 填充 PnP 所需的 2D 角点 (顺序：左上, 右上, 右下, 左下)
                    // 对应 Struct.hpp 中 Light 构造函数获取的 vertices
                    armor.corners = {
                        leftBar.vertices[0],  // 左灯条顶端
                        rightBar.vertices[0], // 右灯条顶端
                        rightBar.vertices[1], // 右灯条底端
                        leftBar.vertices[1]   // 左灯条底端
                    };
                    
                    armor.center = (leftBar.center + rightBar.center) * 0.5f;

                    if (solvePose(armor)) {
                        armors.push_back(armor);
                    }
                }
            }
        }
    }
    return armors;
}

bool ArmorsDetector::solvePose(Armors& armor) {
    // 3. 3D 模型定义 (装甲板实际尺寸为 0.235m x 0.127m)
    static const std::vector<cv::Point3f> armor_3d = {
        {-0.0705f, -0.0625f, 0.0f}, // 左上
        { 0.0705f, -0.0625f, 0.0f}, // 右上
        { 0.0705f,  0.0625f, 0.0f}, // 右下
        {-0.0705f,  0.0625f, 0.0f}  // 左下
    };

    // 4. 从全局 Config 单例获取标定数据
    auto& cfg = Config::getInstance();
    if (cfg.cameraMatrix.empty()) return false;

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(armor_3d, armor.corners, 
                           cfg.cameraMatrix, cfg.distCoeffs, 
                           rvec, tvec, false, cv::SOLVEPNP_IPPE); // IPPE 针对平面目标更稳

    if (success) {
        armor.rvec = rvec.clone();
        armor.tvec = tvec.clone();
        armor.pos = cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        armor.distance = cv::norm(armor.pos);
        
        // 还可增加 Rodrigues 变换计算 pitch/yaw 逻辑
    }
    return success;
}
