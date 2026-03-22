#pragma once
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
};

// 静态成员初始化（必须在类外）
Config* Config::instance_ = nullptr;
std::mutex Config::mutex_;



