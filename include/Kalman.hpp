#ifndef EXTENDED_KALMAN_HPP 
#define EXTENDED_KALMAN_HPP

#include <opencv2/opencv.hpp>
#include <vector>

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
