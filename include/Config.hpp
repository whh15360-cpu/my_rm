<<<<<<< HEAD
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
=======
/*#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <cstdint>
#include <stdexcept>  // 处理文件加载异常
#include <filesystem> // C++17文件系统库，检查配置文件存在性
#include <ament_index_cpp/get_package_share_directory.hpp>

// YOLO检测核心配置
struct YoloConfig
{
    float conf_thred_blue = 0.6f;  // 蓝色装甲板置信度阈值
    float conf_thred_red = 0.2f;   // 红色装甲板置信度阈值
    float nms_thresh = 0.45f;      // NMS非极大值抑制阈值
    int imgsz = 320;               // YOLO输入尺寸（和训练一致）
    std::string model_path;        // ONNX模型路径
};

// 相机参数配置（位姿解算）
struct CameraConfig
{
    // 相机内参（fx,fy,cx,cy）
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    // 畸变参数（k1,k2,p1,p2,k3）
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;
};

// 装甲板物理尺寸配置（位姿解算）
struct ArmorSizeConfig
{
    float width = 0.141f;  // 装甲板宽度（米）
    float height = 0.125f; // 装甲板高度（米）
};

// 应用总配置
struct AppConfig
{
    YoloConfig yolo_config;          // YOLO核心配置
    CameraConfig armor_params ;      // 相机参数
    ArmorSizeConfig armor_size;      // 装甲板物理尺寸
};

namespace YAML
{
    // YOLO配置解析
    template<>
    struct convert<YoloConfig>
    {
        static Node encode(const YoloConfig& rhs)
        {
            Node node;
            node["conf_thred_blue"] = rhs.conf_thred_blue;
            node["conf_thred_red"] = rhs.conf_thred_red;
            node["nms_thresh"] = rhs.nms_thresh;
            node["imgsz"] = rhs.imgsz;
            node["model_path"] = rhs.model_path;
            return node;
        }

        static bool decode(const Node& node, YoloConfig& rhs)
        {
            if (!node.IsMap()) return false;
            // 带默认值的解析（避免配置文件缺项崩溃）
            rhs.conf_thred_blue = node["conf_thred_blue"].as<float>(0.6f);
            rhs.conf_thred_red = node["conf_thred_red"].as<float>(0.2f);
            rhs.nms_thresh = node["nms_thresh"].as<float>(0.45f);
            rhs.imgsz = node["imgsz"].as<int>(320);
            rhs.model_path = node["model_path"].as<std::string>("");
            return true;
        }
    };

    // 相机参数解析
    template<>
    struct convert<CameraConfig>
    {
        static Node encode(const CameraConfig& rhs)
        {
            Node node;
            node["fx"] = rhs.fx;
            node["fy"] = rhs.fy;
            node["cx"] = rhs.cx;
            node["cy"] = rhs.cy;
            node["k1"] = rhs.k1;
            node["k2"] = rhs.k2;
            node["p1"] = rhs.p1;
            node["p2"] = rhs.p2;
            node["k3"] = rhs.k3;
            return node;
        }

        static bool decode(const Node& node, CameraConfig& rhs)
        {
            if (!node.IsMap()) return false;
            // 带默认值解析
            rhs.fx = node["fx"].as<double>(0.0);
            rhs.fy = node["fy"].as<double>(0.0);
            rhs.cx = node["cx"].as<double>(0.0);
            rhs.cy = node["cy"].as<double>(0.0);
            rhs.k1 = node["k1"].as<double>(0.0);
            rhs.k2 = node["k2"].as<double>(0.0);
            rhs.p1 = node["p1"].as<double>(0.0);
            rhs.p2 = node["p2"].as<double>(0.0);
            rhs.k3 = node["k3"].as<double>(0.0);
            return true;
        }
    };

    // 装甲板尺寸解析
    template<>
    struct convert<ArmorSizeConfig>
    {
        static Node encode(const ArmorSizeConfig& rhs)
        {
            Node node;
            node["width"] = rhs.width;
            node["height"] = rhs.height;
            return node;
        }

        static bool decode(const Node& node, ArmorSizeConfig& rhs)
        {
            if (!node.IsMap()) return false;
            rhs.width = node["width"].as<float>(0.141f);
            rhs.height = node["height"].as<float>(0.125f);
            return true;
        }
    };  

    // 总配置解析
    template<>
    struct convert<AppConfig>
    {
        static Node encode(const AppConfig& rhs)
        {            
            Node node;
            node["yolo_config"] = rhs.yolo_config;
            node["armor_params"] = rhs.armor_params;
            node["armor_size"] = rhs.armor_size;
            return node;
        }

        static bool decode(const Node& node, AppConfig& rhs)
        {
            if (!node.IsMap()) return false;
            // 解析各子配置（带默认值）
            rhs.yolo_config = node["yolo_config"].as<YoloConfig>();
            rhs.armor_params = node["armor_params"].as<CameraConfig>();
            rhs.armor_size = node["armor_size"].as<ArmorSizeConfig>();
            return true;
        }
    };  
}

class Config
{
private:
    AppConfig config_;
    static Config* instance_;
    static std::mutex mutex_;
    std::string config_path_;  // 配置文件路径，支持动态加载

    // 私有化构造函数（支持指定配置文件路径）
Config()
{
    // 获取包的共享目录（ROS2标准）
    std::string pkg_path = ament_index_cpp::get_package_share_directory("armor_pose_solver");
    // 拼接配置文件路径
    config_path_ = pkg_path + "/config/config.yaml";
    loadConfig();
}
    explicit Config(const std::string& config_path) : config_path_(config_path)
    {
        loadConfig();
    }   

    // 禁止拷贝和赋值
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    // 抽离配置加载逻辑（ 方便异常处理+重新加载）
    void loadConfig()
    {
        // 校验文件是否存在
        if (!std::filesystem::exists(config_path_))
        {
            throw std::runtime_error("配置文件不存在：" + config_path_);
        }

        try
        {
            YAML::Node root = YAML::LoadFile(config_path_);
            config_ = root.as<AppConfig>();
        }
        catch (const YAML::Exception& e)
        {
            throw std::runtime_error("配置文件解析失败：" + std::string(e.what()));
        }
    }

public:
    // 全局单例入口（支持指定配置文件路径）
    static Config* getInstance(const std::string& config_path = "config/config.yaml")
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ == nullptr)
        {
            instance_ = new Config(config_path);
        }
        return instance_;
    }

    // 重新加载配置（ ROS节点运行中可动态更新）
    void reloadConfig(const std::string& new_path = "")
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!new_path.empty())
        {
            config_path_ = new_path;
        }
        loadConfig();
    }

    // 只读获取配置
    const AppConfig& getConfig() const
    {
        return config_;
    }

    // 析构函数（新增：防止内存泄漏）
    ~Config() = default;
};*/

// 静态成员初始化（必须在类外）
/*Config* Config::instance_ = nullptr;
std::mutex Config::mutex_;*/


#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <cstdint>

// 定义与配置相关的结构体
struct MorphConfig
{
    int kernel_size;
};

struct LightConfig
{
    int area;
    double ratio_min;
    double ratio_max;
    int angle;
};

struct ArmorConfig
{
    double distance_min;
    double distance_max;
    double armor_angle;
    double height_diff;
    double angle_diff;
    double armor_ratio_min;
    double armor_ratio_max;
};

struct KalmanConfig
{
    float processNoisePos;
    float processNoiseVel;
    float measurementNoisePos;
    float measurementNoiseYaw;
    float initialErrorCov;
    float yawnoise;
};

// 在 Congfig.hpp 中添加 UdpConfig 定义（放在其他 Config 结构体附近）
struct UdpConfig
{
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 9870;
};

// 在 AppConfig 结构体中增加成员
struct AppConfig
{
    MorphConfig morph_config;
    LightConfig light_config;
    ArmorConfig armor_config;
    KalmanConfig kalman;
    UdpConfig udp;   // 新增
};

namespace YAML
{
    // 定义 YAML 转换函数
    template<>
    struct convert<MorphConfig>
    {
        static Node encode(const MorphConfig& rhs)
        {
            Node node;
            node["kernel_size"] = rhs.kernel_size;
            return node;
        }

        static bool decode(const Node& node, MorphConfig& rhs)
        {
            if (!node.IsMap() || !node["kernel_size"])
                return false;
            rhs.kernel_size = node["kernel_size"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<LightConfig>
    {
        static Node encode(const LightConfig& rhs)
        {
            Node node;
            node["area"] = rhs.area;
            node["ratio_min"] = rhs.ratio_min;
            node["ratio_max"] = rhs.ratio_max;
            node["angle"] = rhs.angle;
            return node;
        }

        static bool decode(const Node& node, LightConfig& rhs)
        {
            if (!node.IsMap() || !node["area"] || !node["ratio_min"] || !node["ratio_max"] || !node["angle"])
                return false;
            rhs.area = node["area"].as<int>();
            rhs.ratio_min = node["ratio_min"].as<double>();
            rhs.ratio_max = node["ratio_max"].as<double>();
            rhs.angle = node["angle"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<ArmorConfig>
    {
        static Node encode(const ArmorConfig& rhs)
        {
            Node node;
            node["distance_min"] = rhs.distance_min;
            node["distance_max"] = rhs.distance_max;
            node["armor_angle"] = rhs.armor_angle;
            node["height_diff"] = rhs.height_diff;
            node["angle_diff"] = rhs.angle_diff;
            node["armor_ratio_min"] = rhs.armor_ratio_min;
            node["armor_ratio_max"] = rhs.armor_ratio_max;
            return node;
        }

        static bool decode(const Node& node, ArmorConfig& rhs)
        {
            if (!node.IsMap() || !node["distance_min"] || !node["distance_max"] || !node["armor_angle"] || !node["height_diff"] || !node["angle_diff"] || !node["armor_ratio_min"] || !node["armor_ratio_max"])
                return false;
            rhs.distance_min = node["distance_min"].as<double>();
            rhs.distance_max = node["distance_max"].as<double>();
            rhs.armor_angle = node["armor_angle"].as<double>();
            rhs.height_diff = node["height_diff"].as<double>();
            rhs.angle_diff = node["angle_diff"].as<double>();
            rhs.armor_ratio_min = node["armor_ratio_min"].as<double>();
            rhs.armor_ratio_max = node["armor_ratio_max"].as<double>();
            return true;
        }
    };  

    template<>
    struct convert<KalmanConfig>
    {
        static Node encode(const KalmanConfig& rhs)
        {
            Node node;
            node["processNoisePos"] = rhs.processNoisePos;
            node["processNoiseVel"] = rhs.processNoiseVel;
            node["measurementNoisePos"] = rhs.measurementNoisePos;
            node["measurementNoiseYaw"] = rhs.measurementNoiseYaw;
            node["initialErrorCov"] = rhs.initialErrorCov;
            return node;
        }

        static bool decode(const Node& node, KalmanConfig& rhs)
        {
            if(!node.IsMap() || !node["processNoisePos"] || !node["processNoiseVel"] || !node["measurementNoisePos"] || !node["initialErrorCov"])
                return false;
            rhs.processNoisePos = node["processNoisePos"].as<float>();
            rhs.processNoiseVel = node["processNoiseVel"].as<float>();
            rhs.measurementNoisePos = node["measurementNoisePos"].as<float>();
            rhs.measurementNoiseYaw = node["measurementNoiseYaw"].as<float>();
            rhs.initialErrorCov = node["initialErrorCov"].as<float>();
            return true;
        }
    };

    template<>
    struct convert<UdpConfig>
    {
        static Node encode(const UdpConfig& rhs)
        {
            Node node;
            node["enabled"] = rhs.enabled;
            node["host"] = rhs.host;
            node["port"] = rhs.port;
            return node;
        }

        static bool decode(const Node& node, UdpConfig& rhs)
        {
            if (!node.IsMap() || !node["enabled"] || !node["host"] || !node["port"])
                return false;
            rhs.enabled = node["enabled"].as<bool>();
            rhs.host = node["host"].as<std::string>();
            rhs.port = node["port"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<AppConfig>
    {
        static Node encode(const AppConfig& rhs)
        {            
            Node node;
            node["morph_config"] = rhs.morph_config;
            node["light_config"] = rhs.light_config;
            node["armor_config"] = rhs.armor_config;
            node["kalman"] = rhs.kalman;
            node["udp"] = rhs.udp; // 新增
            return node;
        }

        static bool decode(const Node& node, AppConfig& rhs)
        {
            if (!node.IsMap() || !node["morph_config"] || !node["light_config"] || !node["armor_config"])
                return false;
            rhs.morph_config = node["morph_config"].as<MorphConfig>();
            rhs.light_config = node["light_config"].as<LightConfig>();
            rhs.armor_config = node["armor_config"].as<ArmorConfig>();
            rhs.kalman = node["kalman"].as<KalmanConfig>();
            rhs.udp = node["udp"].as<UdpConfig>(); // 新增
            return true;
        }
    };  
}

class Config
{
private:
    AppConfig config_;
    static Config* instance_;
    static std::mutex mutex_;

    // 构造函数私有化，禁止外部实例化
    Config()
    {
        YAML::Node root = YAML::LoadFile("src/congfig.yaml");
        config_ = root.as<AppConfig>();
    }

    // 禁止拷贝构造和赋值操作
    Config(const Config&) = delete; // 禁止拷贝构造
    Config& operator=(const Config&) = delete; // 禁止赋值操作

public:
    // 全局获取入口（线程安全的单例模式）
    static Config* getInstance()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ == nullptr)
        {
            instance_ = new Config();
        }
        return instance_;
    }

    // 只读获取配置
    const AppConfig& getConfig() const
    {
        return config_;
    }
};
>>>>>>> 33463e2bc191262bb0ef0e94a4143a5cba0005c8
