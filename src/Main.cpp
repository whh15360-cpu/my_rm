/*#include "Config.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include "ExtendedKalman.hpp"
#include "Serial.hpp"
#include "DrawTrack.hpp"
#include "Plotter.hpp"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>


int main()
{
    cv::setNumThreads(0);
    cv::setUseOptimized(false);
    
    // 帧计数器（每隔5帧发送一次）
    int frameCounter = 0;
    const int SEND_INTERVAL = 5;

    const AppConfig& appCfg = Config::getInstance()->getConfig();

    std::unique_ptr<tools::Plotter> plotter;
    if (appCfg.udp.enabled)
    {
        plotter = std::make_unique<tools::Plotter>(appCfg.udp.host, appCfg.udp.port);
        std::cout << "Plotter 已启用，目标 " << appCfg.udp.host << ":" << appCfg.udp.port << std::endl;
    }
    else
    {
        std::cout << "Plotter 发送已禁用" << std::endl;
    }

    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "[ERROR] 无法打开calib_result.yml" << std::endl;
        return -1;
    }
    fs["cameraMatrix"] >> camera_matrix;
    fs["distCoeffs"] >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cout << "PnPSolver::solve: 无效的相机内参，跳过位姿求解" << std::endl;
        return -1;
    }
    
    // 检测器
    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);

    // 卡尔曼追踪
    ExtendedKalman tracker;
    bool inited = false;

    cv::Point3f last_valid_position; 
    double last_valid_yaw;
    double last_valid_pitch;           
    cv::Mat last_valid_rvec;     
    
    // // 打开串口（改为 ttyS0）
    // Serial serial;
    // if (!serial.open("/dev/ttyS0", B115200))
    // {
    //     std::cerr << "无法打开串口" << std::endl;
    //     return -1;
    // }

    // 视频源
    cv::VideoCapture cap("src/red1.mp4");
    if (!cap.isOpened())
    { 
        std::cerr << "视频打开失败" << std::endl;
        return -1;
    }
    
    // // 海康工业相机（备选）
    // HikCamera cam;
    // if (!cam.init())
    // {
    //     return -1;
    // }

    cv::namedWindow("Armor Tracker", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Armor Tracker", 1280, 720);
    
    cv::Mat frame;

    double timeStamp = 0.0;
    const double dt = 0.033;

    while (true)
    {
        // 获取一帧图像
        if (!cap.read(frame))
        {
            break;
        }
        
        // // 若使用海康相机，取消注释以下代码
        // if (!cam.getFrame(frame))
        // {
        //     continue;
        // }
        
        if (frame.empty())
        {
            continue;
        }
        
        frameCounter++;  // 每帧递增计数器

        std::vector<Armors> armors = armorsdetector.detect(frame);

        if (!armors.empty())
        {
            const Armors& best = armors[0];

            cv::Point3f pos(
                best.tvec.at<double>(0),
                best.tvec.at<double>(1),
                best.tvec.at<double>(2)
            );

            double yaw = best.yaw;
            double pitch = best.pitch;

            // 保存上一次有效值（用于绘制及 pitch 预测占位）
            last_valid_position = pos;
            last_valid_yaw = yaw;
            last_valid_pitch = pitch;
            last_valid_rvec = best.rvec.clone();

            // ------------------ 卡尔曼更新 ------------------
            if (!inited)
            {
                tracker.init(pos, yaw, timeStamp);
                inited = true;
            }
            else
            {
                tracker.update(pos, yaw, timeStamp);
            }

            // 获取卡尔曼预测值（估计位置和预测偏航角）
            cv::Point3f est_pos = tracker.getEstimatedPosition();
            double pred_yaw_deg = tracker.getPredictedYawDeg();
            double pred_dist = sqrt(est_pos.x*est_pos.x + est_pos.y*est_pos.y + est_pos.z*est_pos.z);

            if (plotter)
            {
                nlohmann::json j;
                j["timestamp"] = timeStamp;
                j["pnp_distance"] = best.distance;
                j["pnp_yaw"] = best.yaw;
                j["pnp_pitch"] = best.pitch;
                j["pnp_roll"] = best.roll;
                j["est_x"] = est_pos.x;
                j["est_y"] = est_pos.y;
                j["est_z"] = est_pos.z;
                j["pred_yaw"] = pred_yaw_deg;
                // 可以添加更多字段，如测量值位置等
                j["meas_x"] = best.tvec.at<double>(0);
                j["meas_y"] = best.tvec.at<double>(1);
                j["meas_z"] = best.tvec.at<double>(2);
                plotter->plot(j);
            }


            // ------------------ 串口发送（每5帧，仅检测到装甲板时）------------------
            // if (frameCounter % SEND_INTERVAL == 0 && serial.is_open())
            // {
            //     char buffer[256];
            //     snprintf(buffer, sizeof(buffer),
            //              "(yaw: %.3f, pitch: %.3f, dist: %.3f, x: %.3f, y: %.3f, z: %.3f)\n",
            //              pred_yaw_deg, last_valid_pitch, pred_dist,
            //              est_pos.x, est_pos.y, est_pos.z);
                
            //     if (!serial.writeString(buffer, false))  // false: 不自动添加换行（已包含\n）
            //     {
            //         std::cout << "[ERROR] 串口发送失败" << std::endl;
            //     }
            // }

            // 绘制跟踪结果
            drawTrack(frame, best, est_pos, pred_yaw_deg, 
                      last_valid_rvec, camera_matrix, dist_coeffs);
        } 
        else
        {
            // 未检测到装甲板：不发送任何串口数据，只进行卡尔曼预测和绘制
            if (inited)
            {
                tracker.predict(timeStamp);
                cv::Point3f est_pos = tracker.getEstimatedPosition();
                double pred_yaw_deg = tracker.getPredictedYawDeg();

                drawTrack(frame, Armors(), est_pos, pred_yaw_deg, 
                          last_valid_rvec, camera_matrix, dist_coeffs);
            }
        }
        
        cv::imshow("Armor Tracker", frame);
        if (cv::waitKey(30) == 27)
        {
            break;
        }

        timeStamp += dt;
    }

    cap.release();
    // cam.release();
    cv::destroyAllWindows();
    // serial.close();
    return 0;
}
*/



#include "Config.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include "ExtendedKalman.hpp"
#include "Serial.hpp"
#include "DrawTrack.hpp"
#include "Plotter.hpp"

// ROS2 必需头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// 引入定义的自定义接口头文件
#include "armor_interfaces/msg/armor_shoot.hpp"

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <atomic>

// 全局图像缓存（线程安全）
std::atomic<bool> g_new_frame(false);
cv::Mat g_frame;

// ROS2 图像订阅回调
void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (!frame.empty()) {
            g_frame = frame.clone();
            g_new_frame = true;
        }
    } catch (...) {}
}

int main(int argc, char** argv)
{
    // 初始化 ROS2 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("armor_tracker_node");
    
    // 1. 订阅者：获取相机图像
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/hik_camera/image_raw", 10, image_callback
    );

    // 2. 发布者：发布解算后的位姿数据（供 STM32 串口节点使用）
    auto pub = node->create_publisher<armor_interfaces::msg::ArmorShoot>(
        "/armor_solver/cmd_shoot", 10
    );

   
    cv::setNumThreads(0);
    cv::setUseOptimized(false);
    
    int frameCounter = 0;
    const int SEND_INTERVAL = 5;

    const AppConfig& appCfg = Config::getInstance()->getConfig();

    std::unique_ptr<tools::Plotter> plotter;
    if (appCfg.udp.enabled)
    {
        plotter = std::make_unique<tools::Plotter>(appCfg.udp.host, appCfg.udp.port);
        std::cout << "Plotter 已启用，目标 " << appCfg.udp.host << ":" << appCfg.udp.port << std::endl;
    }

    cv::Mat camera_matrix, dist_coeffs;
    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "[ERROR] 无法打开calib_result.yml" << std::endl;
        return -1;
    }
    fs["cameraMatrix"] >> camera_matrix;
    fs["distCoeffs"] >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        return -1;
    }
    
    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);
    ExtendedKalman tracker;
    bool inited = false;

    cv::Point3f last_valid_position; 
    cv::Mat last_valid_rvec;     

    cv::namedWindow("Armor Tracker", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Armor Tracker", 1280, 720);
    
    double timeStamp = 0.0;
    const double dt = 0.033;

    //主循环
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (!g_new_frame || g_frame.empty()) {
            continue;
        }

        cv::Mat frame = g_frame.clone();
        g_new_frame = false;

 
        frameCounter++;
        std::vector<Armors> armors = armorsdetector.detect(frame);

        if (!armors.empty())
        {
            const Armors& best = armors[0];

            cv::Point3f pos(
                best.tvec.at<double>(0),
                best.tvec.at<double>(1),
                best.tvec.at<double>(2)
            );

            double yaw = best.yaw;
            last_valid_position = pos;
            last_valid_rvec = best.rvec.clone();

            if (!inited)
            {
                tracker.init(pos, yaw, timeStamp);
                inited = true;
            }
            else
            {
                tracker.update(pos, yaw, timeStamp);
            }

            cv::Point3f est_pos = tracker.getEstimatedPosition();
            double pred_yaw_deg = tracker.getPredictedYawDeg();

            // 填充并发布 ROS2 消息 
            // 只有识别到目标时才发布，或者可以根据需求逻辑决定发布频率
            auto shoot_msg = armor_interfaces::msg::ArmorShoot();
            shoot_msg.x_center = best.x_center;      // 来自Armors 结构体
            shoot_msg.y_center = best.y_center;      //
            shoot_msg.distance = best.distance;      // PnP 解算出的物理距离
            shoot_msg.confidence = 0.5;             // 可以根据算法逻辑给一个置信度评分
            
            pub->publish(shoot_msg); // 正式发布，之后串口节点就能收到它了

            // Plotter 和绘图逻辑
            if (plotter)
            {
                nlohmann::json j;
                j["timestamp"] = timeStamp;
                j["est_x"] = est_pos.x;
                plotter->plot(j);
            }
            drawTrack(frame, best, est_pos, pred_yaw_deg, last_valid_rvec, camera_matrix, dist_coeffs);
        } 
        else
        {
            if (inited)
            {
                tracker.predict(timeStamp);
                cv::Point3f est_pos = tracker.getEstimatedPosition();
                drawTrack(frame, Armors(), est_pos, 0, last_valid_rvec, camera_matrix, dist_coeffs);
            }
        }
        
        cv::imshow("Armor Tracker", frame);
        if (cv::waitKey(1) == 27) break;

        timeStamp += dt;
    }

    // 退出清理
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}

