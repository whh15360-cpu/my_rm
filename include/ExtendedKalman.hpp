/*#ifndef Kalman_hpp
#define Kalman_hpp

#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/video.hpp>

class Kalman 
{
public:
    // 构造函数：dt为采样周期，已知装甲板水平角速度7.33rad/s
    Kalman(double dt, double angular_velocity = 7.33);
    
    // 初始化滤波器状态
    void init(const cv::Point3f& position, double yaw);

    // 预测下一帧状态
    void predict();

    // 更新滤波器状态
    void update(const cv::Point3f& position, double yaw);

    // 获取当前状态估计
    cv::Point3f getPosition() const;
    cv::Point3f getVelocity() const;
    double getYaw() const;
    double getYawRate() const;

private:
    cv::KalmanFilter kf_;
    double dt_; // 采样周期
    double angular_velocity_; // 已知装甲板水平角速度
    cv::Mat state_; // 状态向量 [x, y, z, vx, vy, vz, yaw, yaw_rate]
    cv::Mat measurement_; // 测量向量 [x, y, z, yaw]
};

#endif 
*/


#ifndef EXTENDED_KALMAN_HPP
#define EXTENDED_KALMAN_HPP

#include <opencv2/opencv.hpp>

/*
    六维卡尔曼（拓展卡尔曼）
    状态向量： x, y, z, vx, vy, vz
    观测： x, y, z
    非线性： yaw = atan2(y, x)
*/

class ExtendedKalman
{
public:
    ExtendedKalman();

    void init(const cv::Point3f& position, double yaw, double timeStamp);
    cv::Point3f predict(double timeStamp);
    cv::Point3f update(const cv::Point3f measuredPos, double measuredYaw, double timeStamp);

    cv::Point3f getEstimatedPosition() const;
    float getPredictedYaw() const;
    float getPredictedYawDeg() const;

private:
    void loadParamInConfig();
    void setTransitionMatrix(double dt);
    float clacYawFromXY(float x, float y) const;

    // EKF 核心函数
    void predictState(); // 预测（线性）

    void predictCovariance(); // 协方差预测

    void computeJacobianH(const cv::Mat& state, cv::Mat& H) const; // 计算观测雅可比

    void update(const cv::Mat& z);

private:
    // 6 x 1 状态向量
    cv::Mat m_state;
    // 6 x 6 协方差矩阵
    cv::Mat m_cov;
    // 6 x 6 状态转移矩阵
    cv::Mat m_F;
    // 6 x 6 过程协方差矩阵
    cv::Mat m_Q;
    // 4 x 4 观测噪声协方差(位置+偏航)
    cv::Mat m_R;

    double m_lastTime;
    double m_dt;
    double m_initialized;

    cv::Point3f m_predictedPose;
    float m_predicetedYaw;
};

#endif // EXTENDED_KALMAN_HPP




