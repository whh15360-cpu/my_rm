/*#include "Kalman.hpp"
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
    kf_ = cv::KalmanFilter(8, 4, 0, CV_64F);

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


#include "ExtendedKalman.hpp"
#include "Congfig.hpp"
#include <opencv2/opencv.hpp>

ExtendedKalman::ExtendedKalman()
    : m_lastTime(0),
      m_dt(0),
      m_initialized(false),
      m_predicetedYaw(0.0f)
{
    // 状态向量 6x1 
    m_state = cv::Mat::zeros(6, 1, CV_32F);
    // 协方差矩阵 6x6
    m_cov = cv::Mat::eye(6, 6, CV_32F);
    // 状态转移矩阵 6x6 （初始为单位阵，在预测时更新）
    m_F = cv::Mat::eye(6, 6, CV_32F);
    // 观测噪声矩阵 6x6 （从配置文件加载）
    m_Q = cv::Mat::zeros(6, 6, CV_32F);
    // 观测噪声矩阵 4x4 （位置三维 + yaw）
    m_R = cv::Mat::zeros(4, 4, CV_32F);

    loadParamInConfig();
}

void ExtendedKalman::loadParamInConfig()
{
    const AppConfig& app_cfg = Config::getInstance()->getConfig();
    const KalmanConfig& c = app_cfg.kalman;

    // 过程噪声 Q
    cv::setIdentity(m_Q, cv::Scalar(0));
    m_Q.at<float>(0,0) = c.processNoisePos;
    m_Q.at<float>(1,1) = c.processNoisePos;
    m_Q.at<float>(2,2) = c.processNoisePos;
    m_Q.at<float>(3,3) = c.processNoiseVel;
    m_Q.at<float>(4,4) = c.processNoiseVel;
    m_Q.at<float>(5,5) = c.processNoiseVel;

    // 观测噪声 R（位置和偏航）
    // 假设位置测量噪声与之前相同，偏航噪声单独配置
    float pos_noise = c.measurementNoisePos; // 位置噪声方差
    float yaw_noise = c.measurementNoiseYaw; // 偏航噪声方差（弧度）
    cv::setIdentity(m_R, cv::Scalar(0));
    m_R.at<float>(0,0) = pos_noise;
    m_R.at<float>(1,1) = pos_noise;
    m_R.at<float>(2,2) = pos_noise;
    m_R.at<float>(3,3) = yaw_noise;

    // 初始协方差
    cv::setIdentity(m_cov, cv::Scalar(c.initialErrorCov));

}

void ExtendedKalman::setTransitionMatrix(double dt)
{
    cv::setIdentity(m_F);
    m_F.at<float>(0,3) = dt;
    m_F.at<float>(1,4) = dt;
    m_F.at<float>(2,5) = dt;
}

void ExtendedKalman::init(const cv::Point3f& position, double yaw, double timeStamp)
{
    m_state.at<float>(0) = position.x;
    m_state.at<float>(1) = position.y;
    m_state.at<float>(2) = position.z;
    m_state.at<float>(3) = 0.0f;
    m_state.at<float>(4) = 0.0f;
    m_state.at<float>(5) = 0.0f;

    m_lastTime = timeStamp;
    m_predictedPose = position;
    m_predicetedYaw = static_cast<float>(yaw);
    m_initialized = true;
}

float ExtendedKalman::clacYawFromXY(float x, float y) const
{
    return atan2(y, x);
}

void ExtendedKalman::predictState()
{
    // 状态预测： x_pred = F * x
    m_state = m_F * m_state;
}

void ExtendedKalman::predictCovariance()
{
    // 协方差预测: P_pred = F * P * F^T + Q
    m_cov = m_F * m_cov * m_F.t() + m_Q;
}

cv::Point3f ExtendedKalman::predict(double timeStamp)
{
    if (!m_initialized)
    {
        return cv::Point3f(0, 0, 0);
    }

    // 计算时间差
    if (m_lastTime > 0)
    {
        m_dt = timeStamp - m_lastTime;
        if (m_dt > 0.1)
        {
            m_dt = 0.033;
        }
        if (m_dt < 0.001)
        {
            m_dt = 0.033;
        }
    }
    else
    {
        m_dt = 0.033;
    }

    setTransitionMatrix(m_dt);
    predictState(); // 状态猜测
    predictCovariance(); // 协方差预测

    // 提取预测位置
    m_predictedPose.x = m_state.at<float>(0);
    m_predictedPose.y = m_state.at<float>(1);
    m_predictedPose.z = m_state.at<float>(2);
    m_predicetedYaw = clacYawFromXY(m_predictedPose.x, m_predictedPose.y);

    m_lastTime = timeStamp;
    return m_predictedPose;
}

void ExtendedKalman::computeJacobianH(const cv::Mat& state, cv::Mat& H) const
{
    // 观测向量为[x, y, z, yaw]^T, yaw = atan2(x, y)
    // 雅可比矩阵H大小为4x6
    H = cv::Mat::zeros(4, 6, CV_32F);
    // 前三行直接观测x, y, z
    H.at<float>(0,0) = 1.0f;
    H.at<float>(1,1) = 1.0f;
    H.at<float>(2,2) = 1.0f;

    // 第四行对应yaw对状态向量的偏导
    float x = state.at<float>(0);
    float y = state.at<float>(1);
    float r2 = x*x + y*y;
    if (r2 < 1e-6f)
    {
        r2 = 1e-6f;
    }
    H.at<float>(3,0) = -y / r2;  // ∂yaw/∂x
    H.at<float>(3,1) =  x / r2;  // ∂yaw/∂y
    // ∂yaw/∂z, ∂yaw/∂vx, ∂yaw/∂vy, ∂yaw/∂vz 均为 0

}

void ExtendedKalman::update(const cv::Mat& z)
{
    // 观测向量 z 为4x1 : [x_meas, y_meas, z_meas, yaw_meas]^T

    // 1.计算观测雅可比 H
    cv::Mat H;
    computeJacobianH(m_state, H);

    // 2.计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^ -1
    cv::Mat S = H * m_cov * H.t() + m_R;
    cv::Mat K = m_cov * H.t() * S.inv();

    // 3.计算观测残差
    // 预测观测: h(x_pred) = [x_pred, y_pred, z_pred, atan2(y_pred, x_pred)]^T
    cv::Mat z_pred(4, 1, CV_32F);
    z_pred.at<float>(0) = m_state.at<float>(0);
    z_pred.at<float>(1) = m_state.at<float>(1);
    z_pred.at<float>(2) = m_state.at<float>(2);
    z_pred.at<float>(3) = clacYawFromXY(m_state.at<float>(0), m_state.at<float>(1));

    cv::Mat y = z - z_pred;

    // 处理角度残差的环绕问题(-n到n)
    if (y.at<float>(3) > CV_PI)
    {
        y.at<float>(3) -= 2*CV_PI;
    }
    if (y.at<float>(3) < -CV_PI)
    {
        y.at<float>(3) += 2*CV_PI;
    }
    
    // 4.状态更新
    m_state = m_state + K * y;

    // 5.协方差更新(Joseph form 保证对称性)
    cv::Mat I = cv::Mat::eye(6, 6, CV_32F);
    m_cov = (I - K * H) * m_cov * (I - K * H).t() + K * m_R * K.t();
    
}

cv::Point3f ExtendedKalman::update(const cv::Point3f measuredPos, double measuredYaw, double timeStamp)
{
    if (!m_initialized)
    {
        init(measuredPos, measuredYaw, timeStamp);
        return measuredPos;
    }
    
    // 先预测当前时刻
    predict(timeStamp);

    // 构造观测向量
    cv::Mat z(4, 1, CV_32F);
    z.at<float>(0) = measuredPos.x;
    z.at<float>(1) = measuredPos.y;
    z.at<float>(2) = measuredPos.z;
    z.at<float>(3) = static_cast<float>(measuredYaw);

    // 执行EKF更新
    update(z);

    // 返回滤波后位置
    cv::Point3f estPos(
        m_state.at<float>(0),
        m_state.at<float>(1),
        m_state.at<float>(2)
    );
    m_predictedPose = estPos;
    m_predicetedYaw =clacYawFromXY(estPos.x, estPos.y);

    return estPos;
}

cv::Point3f ExtendedKalman::getEstimatedPosition() const
{
    if (!m_initialized)
    {
        return cv::Point3f(0, 0, 0);
    }
    return cv::Point3f(
        m_state.at<float>(0),
        m_state.at<float>(1),
        m_state.at<float>(2)
    );
}

float ExtendedKalman::getPredictedYaw() const
{
    return m_predicetedYaw;
}

float ExtendedKalman::getPredictedYawDeg() const
{
    return m_predicetedYaw * 180.0f / CV_PI;
}
