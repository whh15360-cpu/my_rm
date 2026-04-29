#include "Kalman.hpp"
#include "Config.hpp"
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
    //const AppConfig& app_cfg = Config::getInstance()->getConfig();
    //const KalmanConfig& c = app_cfg.kalman;
    auto& cfg = Config::getInstance();

    // 过程噪声 Q
    //cv::setIdentity(m_Q, c::Scalar(0));
    m_Q.at<float>(0,0) = cfg.processNoisePos;
    m_Q.at<float>(1,1) = cfg.processNoisePos;
    m_Q.at<float>(2,2) = cfg.processNoisePos;
    m_Q.at<float>(3,3) = cfg.processNoiseVel;
    m_Q.at<float>(4,4) = cfg.processNoiseVel;
    m_Q.at<float>(5,5) = cfg.processNoiseVel;

    // 观测噪声 R（位置和偏航）
    // 假设位置测量噪声与之前相同，偏航噪声单独配置
    float pos_noise = cfg.measurementNoisePos; // 位置噪声方差
    float yaw_noise = cfg.measurementNoiseYaw; // 偏航噪声方差（弧度）
    cv::setIdentity(m_R, cv::Scalar(0));
    m_R.at<float>(0,0) = pos_noise;
    m_R.at<float>(1,1) = pos_noise;
    m_R.at<float>(2,2) = pos_noise;
    m_R.at<float>(3,3) = yaw_noise;

    // 初始协方差
    cv::setIdentity(m_cov, cv::Scalar(cfg.initialErrorCov));

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

    // 计算观测雅可比 H
    cv::Mat H;
    computeJacobianH(m_state, H);

    // 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^ -1
    cv::Mat S = H * m_cov * H.t() + m_R;
    cv::Mat K = m_cov * H.t() * S.inv();

    // 计算观测残差
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
    
    // 状态更新
    m_state = m_state + K * y;

    // 协方差更新(Joseph form 保证对称性)
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
