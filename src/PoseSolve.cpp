/*#include "PoseSolve.hpp"  
#include <opencv4/opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>

// 构造函数：初始化相机参数和配置
ArmorsDetector::ArmorsDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : cameraMatrix_(camera_matrix.clone()), distCoeffs_(dist_coeffs.clone())
{
    // 校验配置单例
    //
    config_ptr_ = Config::getInstance();
    if (config_ptr_ == nullptr) {
        throw std::runtime_error("[ArmorsDetector] 配置单例初始化失败！");
    }
    config_ = &(config_ptr_->getConfig()); // config_是指针类型
    //
}

// ========== 核心接口：YOLO对接（匹配头文件声明） ==========
std::vector<Armors> ArmorsDetector::detect(const cv::Mat& frame, 
                                           const std::vector<cv::Rect>& yoloBoxes, 
                                           const std::vector<float>& confidences, 
                                           const std::vector<int>& classIds)
{
    std::vector<Armors> armors;
    if (frame.empty() || yoloBoxes.empty()) {
        std::cerr << "[detect] 输入为空，跳过" << std::endl;
        return armors;
    }

    // 校验配置有效性
    //if (config_ == nullptr) {
        std::cerr << "[detect] 配置未初始化，使用默认阈值" << std::endl;
        return armors;
    }

    // 读取置信度阈值
    float conf_blue = config_->yolo_config.conf_thred_blue;
    float conf_red = config_->yolo_config.conf_thred_red;
    //

    // 遍历YOLO框
    for (size_t i = 0; i < yoloBoxes.size(); ++i) {
        // 边界检查：避免数组越界
        if (i >= confidences.size() || i >= classIds.size()) {
            std::cerr << "[detect] 索引越界，跳过第" << i << "个框" << std::endl;
            continue;
        }

        int cls = classIds[i];
        float conf = confidences[i];

        // 置信度筛选（蓝0，红1）
        /*if ((cls == 0 && conf < conf_blue) || (cls == 1 && conf < conf_red)) {
            continue;
        }// 置信度筛选（暂时使用固定阈值，后续可对接配置）

        const cv::Rect& box = yoloBoxes[i];
        Armors armor;

        // 初始化装甲板基础信息
        armor.center = cv::Point2f(box.x + box.width/2.0f, box.y + box.height/2.0f);
        armor.boundingRect = cv::RotatedRect(armor.center, cv::Size2f(box.width, box.height), 0.0f);
        armor.classId = cls;
        armor.confidence = conf;

        // 构造四角点（顺时针）
        armor.corners = {
            cv::Point2f(box.x, box.y),                // 左上角
            cv::Point2f(box.x + box.width, box.y),    // 右上角
            cv::Point2f(box.x + box.width, box.y + box.height), // 右下角
            cv::Point2f(box.x, box.y + box.height)    // 左下角
        };

        // 解算位姿并保存有效结果
        if (solveArmorPose(armor)) {
            armors.push_back(armor);
        }
    }

    return armors;
}

// ========== 核心：位姿解算（保留所有异常校验和角点排序） ==========
bool ArmorsDetector::solveArmorPose(Armors& armor)
{
    // 定义装甲板三维参考点（单位：米）
    std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth_/2,  armorHeight_/2, 0.0f},  // 左上角
        { armorWidth_/2,  armorHeight_/2, 0.0f},  // 右上角
        { armorWidth_/2, -armorHeight_/2, 0.0f},  // 右下角
        {-armorWidth_/2, -armorHeight_/2, 0.0f}   // 左下角
    };  //这个是官方定义的装甲板四个角点的三维坐标，单位是米，中心在原点，x轴向右，y轴向下，z轴垂直于板面向外。根据实际情况可以调整坐标系定义，但要确保与图像坐标系一致。

    // 校验角点数量
    std::vector<cv::Point2f> image_points = armor.corners;
    if (image_points.size() != 4)
    {
        std::cerr << "[solveArmorPose] 角点数量不是4个，跳过" << std::endl;
        return false;
    }

    // 顺时针重新排列角点（核心逻辑）
    //int top_left = 0;
    for (int i = 1; i < 4; i++)
    {
        if (image_points[i].x + image_points[i].y <
            image_points[top_left].x + image_points[top_left].y)
        {
            top_left = i;
        }
    }

    int top_right = 0;
    for (int i = 1; i < 4; i++)
    {
        if (image_points[i].x - image_points[i].y >
            image_points[top_right].x - image_points[top_right].y)
        {
            top_right = i;
        }
    }

    int bottom_right = 0;
    for (int i = 1; i < 4; i++)
    {
        if (image_points[i].x + image_points[i].y >
            image_points[bottom_right].x + image_points[bottom_right].y)
        {
            bottom_right = i;
        }
    }
    
    int bottom_left = 6 - top_left - top_right - bottom_right;

    // 重新排列角点
    std::vector<cv::Point2f> ordered_points(4);
    ordered_points[0] = image_points[top_left];
    ordered_points[1] = image_points[top_right];
    ordered_points[2] = image_points[bottom_right];
    ordered_points[3] = image_points[bottom_left];
    //
    
    std::vector<cv::Point2f> ordered_points = image_points;

    // PnP求解（EPNP算法，稳定性更高）
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        armor_3d_points, ordered_points, cameraMatrix_, 
        distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_EPNP
    );
    if (!success) 
    {
        std::cerr << "[solveArmorPose] PnP求解失败" << std::endl;
        return false;
    }

    // 检查 rvec/tvec 有效性（避免NaN/Inf）
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec)) {
        std::cerr << "[solveArmorPose] rvec 或 tvec 含 NaN/Inf，跳过" << std::endl;
        return false;
    }

    // 检查深度合理性（0 < z ≤ 10米）
    double z = tvec.at<double>(2);
    if (z <= 0 || z > 10.0) {
        std::cerr << "[solveArmorPose] 深度 z 异常: " << z << "，跳过" << std::endl;
        return false;
    }

    // 旋转向量转旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    if (!cv::checkRange(R)) {
        std::cerr << "[solveArmorPose] 旋转矩阵 R 含 NaN/Inf，跳过" << std::endl;
        return false;
    }

    // 计算欧拉角（Z-Y-X顺序，适配装甲板场景）
    double pitch = atan2(-R.at<double>(2,0), sqrt(pow(R.at<double>(0,0), 2) + pow(R.at<double>(1,0),2))) * (180.0 / CV_PI);
    double yaw = atan2(R.at<double>(1,0), R.at<double>(0,0)) * (180.0 / CV_PI);
    double roll = atan2(R.at<double>(2,1), R.at<double>(2,2)) * (180.0 / CV_PI);

    // 校验欧拉角有效性
    if (!std::isfinite(pitch) || !std::isfinite(yaw) || !std::isfinite(roll)) {
        std::cerr << "[solveArmorPose] 欧拉角含 NaN/Inf，跳过" << std::endl;
        return false;
    }

    // 角度归一化到 [-180°, 180°]
    armor.pitch = fmod(pitch + 180.0, 360.0) - 180.0;
    armor.yaw = fmod(yaw + 180.0, 360.0) - 180.0;
    armor.roll = fmod(roll + 180.0, 360.0) - 180.0;

    // 计算并校验距离
    double distance = cv::norm(tvec);
    if (!std::isfinite(distance) || distance <= 0 || distance > 10.0) {
        std::cerr << "[solveArmorPose] 距离异常: " << distance << "，跳过" << std::endl;
        return false;
    }
    armor.distance = static_cast<float>(distance); // 转为float匹配结构体
    
    // 保存旋转/平移向量
    armor.rvec = rvec.clone();
    armor.tvec = tvec.clone();

    return true;
}

// ========== 补充：适配头文件的空实现（兼容旧代码） ==========
std::vector<Armors> ArmorsDetector::detectYoloArmors(const cv::Mat&)
{
    std::vector<Armors> armors;
    std::cerr << "[detectYoloArmors] 该接口未实现，建议使用detect(frame, boxes, confs, cls_ids)" << std::endl;
    return armors;
}

std::vector<Armors> ArmorsDetector::detect(const cv::Mat)
{
    std::vector<Armors> armors;
    std::cerr << "[detect] 传统检测接口已废弃，建议使用detect(frame, boxes, confs, cls_ids)" << std::endl;
    return armors;
}

// ========== 补充：绘制函数实现（匹配头文件声明） ==========
void ArmorsDetector::draw(cv::Mat &img, const std::vector<Armors> &armors)
{
    if (img.empty() || armors.empty()) return;

    // 绘制每个装甲板的轮廓和信息
    for (const auto& armor : armors) {
        // 绘制四角点连线（红色）
        if (armor.corners.size() == 4) {
            for (int i = 0; i < 4; ++i) {
                cv::line(img, armor.corners[i], armor.corners[(i+1)%4], cv::Scalar(0,0,255), 2);
            }
        }

        // 绘制中心点（绿色）
        cv::circle(img, armor.center, 3, cv::Scalar(0,255,0), -1);

        // 显示距离和角度
        char text[50];
        snprintf(text, sizeof(text), "Dist: %.2fm Yaw: %.1f°", armor.distance, armor.yaw);
        cv::putText(img, text, cv::Point(armor.center.x + 10, armor.center.y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
    }
}

// ========== 补充：友元函数实现（匹配头文件声明） ==========
void drawFriend(ArmorsDetector& detector, cv::Mat& img, const std::vector<Armors> &armors)
{
    detector.draw(img, armors);
}

// ========== 废弃：删除冗余的detectFromYolo接口） ==========
// 注：已将detectFromYolo的逻辑整合到detect(frame, boxes, confs, cls_ids)中
*/


#include "PoseSlove.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cmath>
#include <limits>

ArmorsDetector::ArmorsDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : config_(Config::getInstance()->getConfig())
{
    cameraMatrix_ = camera_matrix.clone();
    distCoeffs_ = dist_coeffs.clone();
    config_ptr_ = Config::getInstance();
}

bool ArmorsDetector::solveArmorPose(Armors& armor)
{
    // 1. 三维参考点（单位：米）
    std::vector<cv::Point3f> objectPoints = {
        {-armorWidth_/2, -armorHeight_/2, 0.0f}, // 左上
        { armorWidth_/2, -armorHeight_/2, 0.0f}, // 右上
        { armorWidth_/2,  armorHeight_/2, 0.0f}, // 右下
        {-armorWidth_/2,  armorHeight_/2, 0.0f}  // 左下
    };

    // 2. 图像角点（必须与三维点严格对应）
    std::vector<cv::Point2f> imagePoints(4);
    imagePoints[0] = armor.left.center  + cv::Point2f(0, -armor.left.rect.size.height/2);  // 左上
    imagePoints[1] = armor.right.center + cv::Point2f(0, -armor.right.rect.size.height/2); // 右上
    imagePoints[2] = armor.right.center + cv::Point2f(0,  armor.right.rect.size.height/2); // 右下
    imagePoints[3] = armor.left.center  + cv::Point2f(0,  armor.left.rect.size.height/2);  // 左下

    // 3. 使用 solvePnPGeneric 获取 IPPE 双解
    std::vector<cv::Mat> rvecs, tvecs;
    cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix_, distCoeffs_,
                        rvecs, tvecs, false, cv::SOLVEPNP_IPPE);

    if (rvecs.empty() || tvecs.empty())
    {
        std::cerr << "[solveArmorPose] solvePnPGeneric 未返回任何解" << std::endl;
        return false;
    }

    // 4. 选择物体在相机前方 (Z > 0) 的解，并用迭代法精化
    cv::Mat rvec, tvec;
    bool found = false;
    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        if (rvecs[i].empty() || tvecs[i].empty()) continue;

        // 检查 Z 分量是否为正（相机前方）
        if (tvecs[i].at<double>(2) > 0)
        {
            // 用迭代法精化该解
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_,
                         rvecs[i], tvecs[i], true, cv::SOLVEPNP_ITERATIVE);
            rvec = rvecs[i];
            tvec = tvecs[i];
            found = true;
            break;
        }
    }

    if (!found)
    {
        std::cerr << "[solveArmorPose] 未找到有效的IPPE解(Z>0)" << std::endl;
        return false;
    }

    // 5. 有效性检查
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec))
    {
        std::cerr << "[solveArmorPose] rvec 或 tvec 含 NaN/Inf" << std::endl;
        return false;
    }

    double distance = cv::norm(tvec);
    if (distance <= 0 || distance > 10.0)
    {
        std::cerr << "[solveArmorPose] 距离异常: " << distance << std::endl;
        return false;
    }

    // 6. 计算旋转矩阵与欧拉角
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    if (!cv::checkRange(R))
    {
        std::cerr << "[solveArmorPose] 旋转矩阵含 NaN/Inf" << std::endl;
        return false;
    }

    // pitch轴（俯仰角）【y】
    double pitch =  asin(-R.at<double>(2, 0)) * 180.0 / CV_PI;
    // yaw轴（偏航角）【z】
    double yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;
    // roll轴（翻滚角）【x】
    double roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / CV_PI;


    if (!std::isfinite(pitch) || !std::isfinite(yaw) || !std::isfinite(roll))
    {
        std::cerr << "[solveArmorPose] 欧拉角含 NaN/Inf" << std::endl;
        return false;
    }

    // 7. 重投影误差校验（过滤错误解）
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix_, distCoeffs_, projected);
    double reprojErr = cv::norm(imagePoints, projected, cv::NORM_L2) / 4.0;
    if (reprojErr > 5.0)  // 阈值根据图像分辨率调整
    {
        std::cerr << "[solveArmorPose] 重投影误差过大: " << reprojErr << std::endl;
        return false;
    }

    // 8. 角度归一化到 [-180, 180)
    auto normalizeAngle = [](double angle) -> double {
        angle = std::fmod(angle, 360.0);
        if (angle < -180.0) angle += 360.0;
        if (angle >= 180.0) angle -= 360.0;
        return angle;
    };

    armor.pitch = normalizeAngle(pitch);
    armor.yaw   = normalizeAngle(yaw);
    armor.roll  = normalizeAngle(roll);
    armor.distance = distance;
    armor.rvec = rvec.clone();
    armor.tvec = tvec.clone();

    // 调试输出
    std::cout << "最终结果: 距离=" << distance << "m, "
              << "pitch=" << armor.pitch << "°, yaw=" << armor.yaw << "°, roll=" << armor.roll << "°" << std::endl;

    return true;
}

std::vector<Armors> ArmorsDetector::detect(const cv::Mat frame)
{
    std::vector<Armors> armors;

    cv::Mat mask = preprocessImage(frame);
    std::vector<Light> lights = detectLights(mask);
    std::vector<Armors> matchedArmors = matchArmors(lights);

    for (const auto& matched : matchedArmors)
    {
        Armors info = matched;

        if(solveArmorPose(info))
        {
            armors.push_back(info);
        }
    }
    return armors;
}
