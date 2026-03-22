#ifndef Kalman_hpp
#define Kalman_hpp

#include <opencv2/opencv.hpp>
#include <vector>

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

#endif // Kalman_hpp