#include "Kalman.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

// 构造函数：dt为采样周期，angular_velocity默认7.33rad/s（装甲板水平角速度）
Kalman::Kalman(double dt, double angular_velocity)
    : dt_(dt), angular_velocity_(angular_velocity)
{
    // 校验采样周期有效性
    if (dt <= 0) {
        throw std::invalid_argument("[Kalman] 采样周期dt必须大于0，当前值：" + std::to_string(dt));
    }

    // 状态维度8：x,y,z,vx,vy,vz,yaw,yaw_rate | 测量维度4：x,y,z,yaw | 无控制输入
    kf_ = cv::Kalman(8, 4, 0, CV_64F);

    // 初始化状态/测量向量
    state_ = cv::Mat::zeros(8, 1, CV_64F);
    measurement_ = cv::Mat::zeros(4, 1, CV_64F);

    // ========== 1. 状态转移矩阵A（匀速模型） ==========
    cv::Mat A = cv::Mat::eye(8, 8, CV_64F);
    A.at<double>(0, 3) = dt_;  // x' = x + vx*dt
    A.at<double>(1, 4) = dt_;  // y' = y + vy*dt
    A.at<double>(2, 5) = dt_;  // z' = z + vz*dt
    A.at<double>(6, 7) = dt_;  // yaw' = yaw + yaw_rate*dt
    kf_.transitionMatrix = A;

    // ========== 2. 测量矩阵H（仅测量位置和yaw角） ==========
    cv::Mat H = cv::Mat::zeros(4, 8, CV_64F);
    H.at<double>(0, 0) = 1.0;  // 测量x → 状态x
    H.at<double>(1, 1) = 1.0;  // 测量y → 状态y
    H.at<double>(2, 2) = 1.0;  // 测量z → 状态z
    H.at<double>(3, 6) = 1.0;  // 测量yaw → 状态yaw
    kf_.measurementMatrix = H;

    // ========== 3. 过程噪声协方差Q（适配装甲板跟踪） ==========
    cv::Mat Q = cv::Mat::zeros(8, 8, CV_64F);
    // 位置噪声（x/y/z）：装甲板位置变化平缓，噪声小
    Q.at<double>(0, 0) = 1e-4;
    Q.at<double>(1, 1) = 1e-4;
    Q.at<double>(2, 2) = 1e-4;
    // 速度噪声（vx/vy/vz）：匀速假设，噪声中等
    Q.at<double>(3, 3) = 1e-2;
    Q.at<double>(4, 4) = 1e-2;
    Q.at<double>(5, 5) = 1e-2;
    // yaw角噪声：装甲板旋转平缓，噪声小
    Q.at<double>(6, 6) = 5e-3;
    // yaw_rate噪声：已知角速度7.33rad/s，噪声中等
    Q.at<double>(7, 7) = 1e-2;
    kf_.processNoiseCov = Q;

    // ========== 4. 测量噪声协方差R（适配相机测量精度） ==========
    cv::Mat R = cv::Mat::zeros(4, 4, CV_64F);
    R.at<double>(0, 0) = 1e-4;  // x测量噪声（≈1cm误差）
    R.at<double>(1, 1) = 1e-4;  // y测量噪声
    R.at<double>(2, 2) = 1e-4;  // z测量噪声
    R.at<double>(3, 3) = pow(0.5 * CV_PI / 180.0, 2);  // yaw测量噪声（0.5度转弧度）
    kf_.measurementNoiseCov = R;

    // ========== 5. 初始后验协方差P ==========
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(0.1));  // 初始不确定度10%
}

// 初始化滤波器状态
void Kalman::init(const cv::Point3f& position, double yaw)
{
    // 校验输入有效性
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
        std::cerr << "[Kalman::init] 位置参数含NaN/Inf，初始化失败" << std::endl;
        return;
    }
    if (!std::isfinite(yaw)) {
        std::cerr << "[Kalman::init] yaw角含NaN/Inf，初始化失败" << std::endl;
        return;
    }

    // 初始化状态向量
    state_.at<double>(0) = position.x;    // x位置
    state_.at<double>(1) = position.y;    // y位置
    state_.at<double>(2) = position.z;    // z位置
    state_.at<double>(3) = 0.0;           // 初始vx=0
    state_.at<double>(4) = 0.0;           // 初始vy=0
    state_.at<double>(5) = 0.0;           // 初始vz=0
    state_.at<double>(6) = yaw;           // 初始yaw角（弧度）
    state_.at<double>(7) = angular_velocity_;  // 初始yaw_rate=7.33rad/s

    // 更新后验状态
    kf_.statePost = state_.clone();
    std::cout << "[Kalman::init] 初始化完成 | 位置：(" 
              << position.x << "," << position.y << "," << position.z 
              << ") | yaw：" << yaw * 180 / CV_PI << "° | 初始角速度：" << angular_velocity_ << "rad/s" << std::endl;
}

// 预测下一帧状态
void Kalman::predict()
{
    try {
        cv::Mat prediction = kf_.predict();
        // 校验预测结果有效性
        if (!cv::checkRange(prediction)) {
            std::cerr << "[Kalman::predict] 预测结果含NaN/Inf，使用上一帧状态" << std::endl;
            return;
        }
        state_ = prediction.clone();
    } catch (const cv::Exception& e) {
        std::cerr << "[Kalman::predict] 预测失败：" << e.what() << std::endl;
    }
}   

// 更新滤波器状态
void Kalman::update(const cv::Point3f& position, double yaw)
{
    // 校验测量值有效性
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || !std::isfinite(yaw)) {
        std::cerr << "[Kalman::update] 测量值含NaN/Inf，跳过更新" << std::endl;
        return;
    }

    // 填充测量向量
    measurement_.at<double>(0) = position.x;
    measurement_.at<double>(1) = position.y;
    measurement_.at<double>(2) = position.z;
    measurement_.at<double>(3) = yaw;

    // 执行状态更新
    try {
        cv::Mat estimated = kf_.correct(measurement_);
        // 校验更新结果有效性
        if (!cv::checkRange(estimated)) {
            std::cerr << "[Kalman::update] 更新结果含NaN/Inf，使用上一帧状态" << std::endl;
            return;
        }
        state_ = estimated.clone();
    } catch (const cv::Exception& e) {
        std::cerr << "[Kalman::update] 更新失败：" << e.what() << std::endl;
    }
}

// 获取滤波后的位置
cv::Point3f Kalman::getPosition() const
{
    return cv::Point3f(
        static_cast<float>(state_.at<double>(0)),
        static_cast<float>(state_.at<double>(1)),
        static_cast<float>(state_.at<double>(2))
    );
}

// 获取滤波后的速度
cv::Point3f Kalman::getVelocity() const
{
    return cv::Point3f(
        static_cast<float>(state_.at<double>(3)),
        static_cast<float>(state_.at<double>(4)),
        static_cast<float>(state_.at<double>(5))
    );
}  

// 获取滤波后的yaw角（弧度，归一化到[-π, π]）
double Kalman::getYaw() const
{
    double yaw = state_.at<double>(6);
    return fmod(yaw + M_PI, 2 * M_PI) - M_PI;
}

// 获取滤波后的yaw角速度（rad/s）
double Kalman::getYawRate() const
{
    return state_.at<double>(7);
}

// 重置滤波器（跟踪丢失后重新初始化）
/*void Kalman::reset()
{
    state_ = cv::Mat::zeros(8, 1, CV_64F);
    measurement_ = cv::Mat::zeros(4, 1, CV_64F);
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(0.1));
    std::cout << "[Kalman::reset] 滤波器已重置" << std::endl;
}*/
