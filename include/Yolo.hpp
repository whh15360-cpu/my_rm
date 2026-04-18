/*#ifndef YOLO_HPP
#define YOLO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

// ====================== 配置常量（可根据需求修改）======================
// 模型输入尺寸（和训练时一致）
const int INPUT_WIDTH = 320;
const int INPUT_HEIGHT = 320;
// 置信度阈值（过滤低置信度检测框）
const float CONF_THRESHOLD = 0.5;
// NMS非极大值抑制阈值（去重重叠框）
const float NMS_THRESHOLD = 0.45;
// 图像归一化参数（YOLO默认：像素/255，RGB通道）
const float SCALE = 1.0 / 255.0;
const cv::Scalar MEAN = cv::Scalar(0, 0, 0);

// ====================== 检测结果结构体 ======================
struct DetectionResult {
    int class_id;          // 类别ID
    std::string class_name;// 类别名称
    float confidence;      // 置信度
    cv::Rect bbox;         // 检测框（x,y,w,h）
};

// ====================== YOLO核心类（CPU版）======================
class YOLOSolver {
public:
    // 构造函数：加载模型和类别文件
    YOLOSolver(const std::string& model_path, const std::string& labels_path);
    
    // 析构函数：释放资源
    ~YOLOSolver();
    
    // 核心方法：检测单张图像
    std::vector<DetectionResult> detect(const cv::Mat& img);
    
    // 辅助方法：绘制检测结果到图像
    void drawResults(cv::Mat& img, const std::vector<DetectionResult>& results);

private:
    // 私有成员：模型和类别数据
    cv::dnn::Net net_;                  // OpenCV DNN网络
    std::vector<std::string> labels_;   // 类别名称列表
    
    // 私有方法：图像预处理
    cv::Mat preprocessImage(const cv::Mat& img);
    
    // 私有方法：解析模型输出
    std::vector<DetectionResult> parseOutput(
        const cv::Mat& output, 
        const cv::Size& img_size
    );
    
    // 私有方法：加载类别名称
    std::vector<std::string> loadLabels(const std::string& path);
};

#endif 
*/

