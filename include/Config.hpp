/*#ifndef PARAM_LOADER_HPP
#define PARAM_LOADER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

class ParamLoader {
public:
    // 静态函数：传入文件路径，通过引用返回矩阵
    static bool loadCameraParams(const std::string& path, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "[ERROR] 无法打开标定文件: " << path << std::endl;
            return false;
        }

        // 直接从 YAML 的 Key 读取
        fs["cameraMatrix"] >> camera_matrix;
        fs["distCoeffs"] >> dist_coeffs;
        fs.release();

        if (camera_matrix.empty() || dist_coeffs.empty()) {
            std::cerr << "[ERROR] 标定文件内容为空或 Key 不匹配！" << std::endl;
            return false;
        }

        return true;
    }
};

#endif
*/


#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>


 //负责从标定文件（calib_result.yml）中读取并持有高频复用的相机参数
 
class Config {
public:
    // 获取全局唯一实例
    static Config& getInstance() {
        static Config instance;
        return instance;
    }

    
    bool loadFromFile(const std::string& path) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "[ERROR] Config: 无法打开标定文件 -> " << path << std::endl;
            return false;
        }

        // 读取 YAML 中的 cameraMatrix 和 distCoeffs 节点
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;
        
        // 读取 kalman_config 嵌套节点
        cv::FileNode kalman = fs["kalman_config"];
        if (!kalman.empty()) {
            
            kalman["processNoisePos"] >> processNoisePos;
            kalman["processNoiseVel"] >> processNoiseVel;
            kalman["measurementNoisePos"] >> measurementNoisePos;
            kalman["measurementNoiseYaw"] >> measurementNoiseYaw;
            kalman["initialErrorCov"] >> initialErrorCov;
            std::cout << "[INFO] Config: Kalman数据 加载成功。" << std::endl;
        } 
        else {
            std::cout << "[WARN] Config: 未找到 kalman_config 节点，使用默认值" << std::endl;
        }
        
        fs.release();

        if (cameraMatrix.empty() || distCoeffs.empty()) {
            std::cerr << "[ERROR] Config: 标定文件关键数据缺失！" << std::endl;
            return false;
        }

        return true;
    }
       

    // 公有成员，方便其他模块直接引用
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    
    // 设置默认值，防止 YAML 读取失败时卡尔曼滤波变量的数据为脏数据
    float processNoisePos = 0.1f;
    float processNoiseVel = 0.5f;
    float measurementNoisePos = 0.01f;
    float measurementNoiseYaw = 5.0f;
    float initialErrorCov = 0.5f;
    
    std::string default_path = "/home/hh/ros2_ws/src/old/config/calib_result.yml";

private:
    // 私有化构造函数，防止外部实例化
    Config() {}
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
};

#endif // CONFIG_HPP
