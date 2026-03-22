// draw.cpp
#include <opencv2/opencv.hpp>
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
}
