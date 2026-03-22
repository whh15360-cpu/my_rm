/*#include "Config.hpp"  
#include "Struct.hpp"
#include "PoseSolve.hpp"  
#include "Kalman.hpp"
#include <cstdint>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>

// 可视化跟踪结果（检测框+预测框+坐标信息）
void drawTrack(cv::Mat& img, const Armors& detected_armor, const cv::Point3f& predict_position, double predict_yaw)
{
    if (img.empty()) {
        std::cerr << "[drawTrack] 输入图像为空，跳过绘制" << std::endl;
        return;
    }

    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    // 读取标定文件
    std::vector<std::string> calib_paths = {
    // 1. ROS2包安装后的配置路径（推荐）
    //ament_index_cpp::get_package_share_directory("armor_pose_solver") + "/config/calib_result.yml",
    // 2. 源码目录路径（开发阶段）
    "/home/hh/ros2_ws/src/armor_pose_solver/config/armor_params.yaml",
    // 3. 当前工作目录（兜底）
    "config/armor_params.yaml",
    "armor_params.yaml"
};
    bool calib_loaded = false;
    for (const auto& path : calib_paths) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["cameraMatrix"] >> camera_matrix;
            fs["distCoeffs"] >> dist_coeffs;
            fs.release();
            calib_loaded = true;
            break;
        }
    }
    if (!calib_loaded) {
        std::cerr << "[ERROR] 无法打开相机标定文件" << std::endl;
        return;
    }

    // 校验相机内参有效性
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[ERROR] 相机内参无效（非3x3矩阵）" << std::endl;
        return;
    }

    // 1. 绘制检测到的装甲板轮廓 (红色)
    if (!detected_armor.corners.empty()) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> pts;
        for (const auto& p : detected_armor.corners) {
            pts.emplace_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
        }
        contours.push_back(pts);
        cv::drawContours(img, contours, 0, cv::Scalar(0, 0, 255), 2); // 红色轮廓
    }

    // 2. 校验预测参数有效性
    if (std::isnan(predict_position.x) || std::isnan(predict_position.y) || std::isnan(predict_position.z) ||
        std::isnan(predict_yaw) || predict_position.z <= 0) {
        std::cerr << "[drawTrack] 预测参数无效（NaN/深度≤0），跳过预测框绘制" << std::endl;
        
        // 仅显示错误提示
        cv::putText(img, "Predict: Invalid Data", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return;
    }

    // 3. 定义装甲板三维参考点 (固定尺寸)
    const float armorWidth = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2, -armorHeight/2, 0.0f},
        {-armorWidth/2, -armorHeight/2, 0.0f}
    };

    // 4. 构造预测的旋转向量（yaw绕Z轴旋转，弧度）
    double yaw_rad = predict_yaw * CV_PI / 180.0;
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        cos(yaw_rad), -sin(yaw_rad), 0,
        sin(yaw_rad),  cos(yaw_rad), 0,
        0,             0,            1);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << predict_position.x, predict_position.y, predict_position.z);

    // 5. 投影预测的3D点到2D图像平面
    std::vector<cv::Point2f> predict_image_points;
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix, dist_coeffs, predict_image_points);

    // 6. 绘制预测的装甲板轮廓 (绿色)
    if (predict_image_points.size() == 4) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> pts;
        for (const auto& p : predict_image_points) {
            // 限制点在图像范围内
            int x = std::clamp(static_cast<int>(p.x), 0, img.cols - 1);
            int y = std::clamp(static_cast<int>(p.y), 0, img.rows - 1);
            pts.emplace_back(x, y);
        }
        contours.push_back(pts);
        cv::drawContours(img, contours, 0, cv::Scalar(0, 255, 0), 2); // 绿色轮廓
    } else {
        std::cerr << "[drawTrack] 投影点数量异常：" << predict_image_points.size() << std::endl;
    }
    
    // 7. 显示预测坐标和角度
    char text[100];
    snprintf(text, sizeof(text), "Predict: (%.2f, %.2f, %.2f) Yaw: %.1f°", 
             predict_position.x, predict_position.y, predict_position.z, predict_yaw);
    cv::putText(img, text, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

// 模拟YOLO检测接口（实际使用时替换为真实YOLO推理）
void mockYoloDetect(const cv::Mat& frame, std::vector<cv::Rect>& boxes, std::vector<float>& confs, std::vector<int>& cls_ids)
{
    // 这里仅做示例，实际需要替换为你的YOLO推理代码
    // 输出：boxes=检测框，confs=置信度，cls_ids=类别（0=蓝，1=红）
    boxes.clear();
    confs.clear();
    cls_ids.clear();

    // 模拟检测到一个装甲板（仅示例）
    boxes.emplace_back(200, 150, 80, 60); // x,y,w,h
    confs.emplace_back(0.95);             // 置信度
    cls_ids.emplace_back(1);              // 红色装甲板
}



int main()
{
    // 1. 加载相机标定参数
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<std::string> calib_paths = {
        "src/calib_result.yml",
        "../src/calib_result.yml",
        "../../src/calib_result.yml",
        "calib_result.yml"
    };
    bool calib_loaded = false;
    for (const auto& path : calib_paths) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["cameraMatrix"] >> camera_matrix;
            fs["distCoeffs"] >> dist_coeffs;
            fs.release();
            calib_loaded = true;
            break;
        }
    }
    if (!calib_loaded) {
        std::cerr << "[ERROR] 无法加载相机标定文件！" << std::endl;
        return -1;
    }
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[ERROR] 相机内参无效！" << std::endl;
        return -1;
    }

    // 2. 初始化检测器和卡尔曼滤波器
    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);
    Kalman tracker(0.033, 7.33); // 采样周期0.033s（30帧），初始角速度7.33rad/s
    bool tracker_inited = false;
    cv::Point3f last_valid_position;
    double last_valid_yaw = 0.0;

    // 3. 打开视频/摄像头
    cv::VideoCapture cap;
    // 可选1：打开视频文件
    if (!cap.open("/home/hh/ros2_ws/src/armor_pose_solver/config/armor5blue.mp4")) {
        std::cerr << "[WARNING] 视频文件打开失败，尝试打开摄像头..." << std::endl;
        // 可选2：打开摄像头
        if (!cap.open(0)) {
            std::cerr << "[ERROR] 摄像头打开失败！" << std::endl;
            return -1;
        }
    }

    // 4. 主循环
    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) {
            std::cout << "视频播放完毕/无有效帧！" << std::endl;
            break;
        }

        // ===================== 核心流程 =====================
        // Step 1: YOLO检测（替换为你的真实YOLO代码）
        std::vector<cv::Rect> yolo_boxes;
        std::vector<float> yolo_confs;
        std::vector<int> yolo_cls_ids;
        mockYoloDetect(frame, yolo_boxes, yolo_confs, yolo_cls_ids); // 替换为真实YOLO推理

        // Step 2: 位姿解算（从YOLO框解算装甲板位姿）
        std::vector<Armors> armors = armorsdetector.detect(frame, yolo_boxes, yolo_confs, yolo_cls_ids);

        // Step 3: 卡尔曼滤波跟踪
        if (!armors.empty()) {
            // 取置信度最高的装甲板
            Armors best_armor = armors[0];

            // 校验位姿有效性
            bool pose_valid = !best_armor.tvec.empty() && cv::checkRange(best_armor.tvec) && 
                              std::isfinite(best_armor.yaw) && best_armor.distance > 0;

            if (pose_valid) {
                // 位姿有效：构造3D位置
                cv::Point3f detect_point(
                    static_cast<float>(best_armor.tvec.at<double>(0)),
                    static_cast<float>(best_armor.tvec.at<double>(1)),
                    static_cast<float>(best_armor.tvec.at<double>(2))
                );
                double detect_yaw = best_armor.yaw;

                // 初始化/更新卡尔曼滤波器
                if (!tracker_inited) {
                    tracker.init(detect_point, detect_yaw);
                    tracker_inited = true;
                    last_valid_position = detect_point;
                    last_valid_yaw = detect_yaw;
                } else {
                    tracker.predict();
                    tracker.update(detect_point, detect_yaw);
                    last_valid_position = detect_point;
                    last_valid_yaw = detect_yaw;
                }

                // 获取预测结果并可视化
                cv::Point3f predict_point = tracker.getPosition();
                double predict_yaw = tracker.getYaw();
                drawTrack(frame, best_armor, predict_point, predict_yaw);
            } else {
                // 位姿无效：仅预测
                std::cerr << "[WARNING] 装甲板位姿无效，仅执行预测" << std::endl;
                if (tracker_inited) {
                    tracker.predict();
                    cv::Point3f predict_point = tracker.getPosition();
                    double predict_yaw = tracker.getYaw();
                    drawTrack(frame, best_armor, predict_point, predict_yaw);
                }
            }
        } else {
            // 未检测到装甲板：仅预测（如果已初始化）
            if (tracker_inited) {
                tracker.predict();
                cv::Point3f predict_point = tracker.getPosition();
                double predict_yaw = tracker.getYaw();
                drawTrack(frame, Armors(), predict_point, predict_yaw);
            }
        }

        // ===================== 可视化 =====================
        cv::imshow("Armor Tracker (YOLO+Kalman)", frame);
        int key = cv::waitKey(30); // 30ms ≈ 30帧/秒
        if (key == 113) { // q键退出
            std::cout << "用户退出程序！" << std::endl;
            break;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}*/


#include "Config.hpp"  
#include "Struct.hpp"
#include "PoseSolve.hpp"  
#include "Kalman.hpp"
#include <cstdint>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>



const std::string YOLO_MODEL_PATH = "/home/hh/ros2_ws/src/armor_pose_solver/models/yolov8_armor.onnx"; // YOLO ONNX模型路径
const std::string YOLO_LABELS_PATH = "/home/hh/ros2_ws/src/armor_pose_solver/config/armor.names";       // 类别文件（0=蓝装甲板，1=红装甲板）
const int YOLO_INPUT_WIDTH = 320;  // 模型输入宽度
const int YOLO_INPUT_HEIGHT = 320; // 模型输入高度
const float YOLO_CONF_THRESH = 0.5; // 置信度阈值
const float YOLO_NMS_THRESH = 0.45; // NMS非极大值抑制阈值

// YOLO检测结果结构体
struct YoloDetection {
    cv::Rect bbox;      // 检测框(x,y,w,h)
    float confidence;   // 置信度
    int class_id;       // 类别ID（0=蓝，1=红）
};

// 加载YOLO类别名称
std::vector<std::string> loadYoloLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[YOLO] 无法打开类别文件: " << path << std::endl;
        return labels;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) labels.push_back(line);
    }
    file.close();
    return labels;
}

// YOLO图像预处理
cv::Mat yoloPreprocess(const cv::Mat& img) {
    cv::Mat blob;
    cv::dnn::blobFromImage(
        img,
        blob,
        1.0 / 255.0,          // 归一化系数
        cv::Size(YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT), // 输入尺寸
        cv::Scalar(0, 0, 0),  // 均值
        true,                 // 交换RB通道（OpenCV BGR→RGB）
        false                 // 不裁剪，保持比例填充
    );
    return blob;
}

// 解析YOLO输出结果
std::vector<YoloDetection> parseYoloOutput(const cv::Mat& output, const cv::Size& img_size) {
    std::vector<YoloDetection> detections;
    int num_detections = output.size[2];
    int num_classes = output.size[1] - 4;

    for (int i = 0; i < num_detections; ++i) {
        float* data = (float*)output.ptr(0, 0, i);
        // 归一化坐标 (cx, cy, w, h)
        float cx = data[0];
        float cy = data[1];
        float w = data[2];
        float h = data[3];
        // 类别置信度
        float* scores = data + 4;
        float max_score = *std::max_element(scores, scores + num_classes);
        int class_id = std::max_element(scores, scores + num_classes) - scores;

        // 过滤低置信度
        if (max_score < YOLO_CONF_THRESH) continue;

        // 转换为原始图像像素坐标
        float x = (cx - w / 2) * img_size.width;
        float y = (cy - h / 2) * img_size.height;
        float width = w * img_size.width;
        float height = h * img_size.height;

        // 限制坐标在图像范围内
        x = std::clamp(x, 0.0f, (float)img_size.width - 1);
        y = std::clamp(y, 0.0f, (float)img_size.height - 1);
        width = std::clamp(width, 1.0f, (float)img_size.width - x);
        height = std::clamp(height, 1.0f, (float)img_size.height - y);

        // 保存检测结果
        detections.push_back({
            cv::Rect(static_cast<int>(x), static_cast<int>(y), static_cast<int>(width), static_cast<int>(height)),
            max_score,
            class_id
        });
    }

    // NMS非极大值抑制去重
    std::vector<int> indices;
    std::vector<cv::Rect> bboxes;
    std::vector<float> confidences;
    for (const auto& det : detections) {
        bboxes.push_back(det.bbox);
        confidences.push_back(det.confidence);
    }
    cv::dnn::NMSBoxes(bboxes, confidences, YOLO_CONF_THRESH, YOLO_NMS_THRESH, indices);

    // 筛选NMS后的结果
    std::vector<YoloDetection> final_detections;
    for (int idx : indices) {
        final_detections.push_back(detections[idx]);
    }
    return final_detections;
}


void realYoloDetect(const cv::Mat& frame, std::vector<cv::Rect>& boxes, std::vector<float>& confs, std::vector<int>& cls_ids, cv::dnn::Net& yolo_net) {
    boxes.clear();
    confs.clear();
    cls_ids.clear();

    // 1. 图像预处理
    cv::Mat blob = yoloPreprocess(frame);

    // 2. YOLO推理
    yolo_net.setInput(blob);
    std::vector<cv::Mat> outputs;
    yolo_net.forward(outputs, yolo_net.getUnconnectedOutLayersNames());

    // 3. 解析输出
    std::vector<YoloDetection> detections = parseYoloOutput(outputs[0], frame.size());

    // 4. 转换为原有格式（适配装甲板检测流程）
    for (const auto& det : detections) {
        boxes.push_back(det.bbox);
        confs.push_back(det.confidence);
        cls_ids.push_back(det.class_id); // 0=蓝，1=红（和你的装甲板类别对应）
    }

    // 打印检测结果（调试用）
    std::cout << "[YOLO] 检测到 " << boxes.size() << " 个装甲板" << std::endl;
}

// 可视化跟踪结果（检测框+预测框+坐标信息）
void drawTrack(cv::Mat& img, const Armors& detected_armor, const cv::Point3f& predict_position, double predict_yaw)
{
    if (img.empty()) {
        std::cerr << "[drawTrack] 输入图像为空，跳过绘制" << std::endl;
        return;
    }

    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    // 读取标定文件
    std::vector<std::string> calib_paths = {
    // 1. ROS2包安装后的配置路径（推荐）
    //ament_index_cpp::get_package_share_directory("armor_pose_solver") + "/config/calib_result.yml",
    // 2. 源码目录路径（开发阶段）
    "/home/hh/ros2_ws/src/armor_pose_solver/config/armor_params.yaml",
    // 3. 当前工作目录（兜底）
    "config/armor_params.yaml",
    "armor_params.yaml"
};
    bool calib_loaded = false;
    for (const auto& path : calib_paths) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["cameraMatrix"] >> camera_matrix;
            fs["distCoeffs"] >> dist_coeffs;
            fs.release();
            calib_loaded = true;
            break;
        }
    }
    if (!calib_loaded) {
        std::cerr << "[ERROR] 无法打开相机标定文件" << std::endl;
        return;
    }

    // 校验相机内参有效性
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[ERROR] 相机内参无效（非3x3矩阵）" << std::endl;
        return;
    }

    // 1. 绘制检测到的装甲板轮廓 (红色)
    if (!detected_armor.corners.empty()) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> pts;
        for (const auto& p : detected_armor.corners) {
            pts.emplace_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
        }
        contours.push_back(pts);
        cv::drawContours(img, contours, 0, cv::Scalar(0, 0, 255), 2); // 红色轮廓
    }

    // 2. 校验预测参数有效性
    if (std::isnan(predict_position.x) || std::isnan(predict_position.y) || std::isnan(predict_position.z) ||
        std::isnan(predict_yaw) || predict_position.z <= 0) {
        std::cerr << "[drawTrack] 预测参数无效（NaN/深度≤0），跳过预测框绘制" << std::endl;
        
        // 仅显示错误提示
        cv::putText(img, "Predict: Invalid Data", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return;
    }

    // 3. 定义装甲板三维参考点 (固定尺寸)
    const float armorWidth = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2, -armorHeight/2, 0.0f},
        {-armorWidth/2, -armorHeight/2, 0.0f}
    };

    // 4. 构造预测的旋转向量（yaw绕Z轴旋转，弧度）
    double yaw_rad = predict_yaw * CV_PI / 180.0;
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        cos(yaw_rad), -sin(yaw_rad), 0,
        sin(yaw_rad),  cos(yaw_rad), 0,
        0,             0,            1);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << predict_position.x, predict_position.y, predict_position.z);

    // 5. 投影预测的3D点到2D图像平面
    std::vector<cv::Point2f> predict_image_points;
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix, dist_coeffs, predict_image_points);

    // 6. 绘制预测的装甲板轮廓 (绿色)
    if (predict_image_points.size() == 4) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> pts;
        for (const auto& p : predict_image_points) {
            // 限制点在图像范围内
            int x = std::clamp(static_cast<int>(p.x), 0, img.cols - 1);
            int y = std::clamp(static_cast<int>(p.y), 0, img.rows - 1);
            pts.emplace_back(x, y);
        }
        contours.push_back(pts);
        cv::drawContours(img, contours, 0, cv::Scalar(0, 255, 0), 2); // 绿色轮廓
    } else {
        std::cerr << "[drawTrack] 投影点数量异常：" << predict_image_points.size() << std::endl;
    }
    
    // 7. 显示预测坐标和角度
    char text[100];
    snprintf(text, sizeof(text), "Predict: (%.2f, %.2f, %.2f) Yaw: %.1f°", 
             predict_position.x, predict_position.y, predict_position.z, predict_yaw);
    cv::putText(img, text, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}


int main()
{
    cv::dnn::Net yolo_net = cv::dnn::readNetFromONNX(YOLO_MODEL_PATH);
    if (yolo_net.empty()) {
        std::cerr << "[ERROR] 无法加载YOLO模型: " << YOLO_MODEL_PATH << std::endl;
        return -1;
    }
    // 强制使用CPU推理（核心！禁用GPU）
    yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    std::cout << "[YOLO] 模型加载成功（CPU模式）: " << YOLO_MODEL_PATH << std::endl;

    // 加载YOLO类别名称（调试用）
    std::vector<std::string> yolo_labels = loadYoloLabels(YOLO_LABELS_PATH);
    if (yolo_labels.empty()) {
        std::cerr << "[WARNING] YOLO类别文件加载失败，不影响检测但无法显示类别名称" << std::endl;
    }

    // 1. 加载相机标定参数
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<std::string> calib_paths = {
        "src/calib_result.yml",
        "../src/calib_result.yml",
        "../../src/calib_result.yml",
        "calib_result.yml"
    };
    bool calib_loaded = false;
    for (const auto& path : calib_paths) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["cameraMatrix"] >> camera_matrix;
            fs["distCoeffs"] >> dist_coeffs;
            fs.release();
            calib_loaded = true;
            break;
        }
    }
    if (!calib_loaded) {
        std::cerr << "[ERROR] 无法加载相机标定文件！" << std::endl;
        return -1;
    }
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[ERROR] 相机内参无效！" << std::endl;
        return -1;
    }

    // 2. 初始化检测器和卡尔曼滤波器
    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);
    Kalman tracker(0.033, 7.33); // 采样周期0.033s（30帧），初始角速度7.33rad/s
    bool tracker_inited = false;
    cv::Point3f last_valid_position;
    double last_valid_yaw = 0.0;

    // 3. 打开视频/摄像头
    cv::VideoCapture cap;
    // 可选1：打开视频文件
    if (!cap.open("/home/hh/ros2_ws/src/armor_pose_solver/config/armor5blue.mp4")) {
        std::cerr << "[WARNING] 视频文件打开失败，尝试打开摄像头..." << std::endl;
        // 可选2：打开摄像头
        if (!cap.open(0)) {
            std::cerr << "[ERROR] 摄像头打开失败！" << std::endl;
            return -1;
        }
    }

    // 4. 主循环
    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) {
            std::cout << "视频播放完毕/无有效帧！" << std::endl;
            break;
        }


        // Step 1: YOLO检测
        std::vector<cv::Rect> yolo_boxes;
        std::vector<float> yolo_confs;
        std::vector<int> yolo_cls_ids;
        realYoloDetect(frame, yolo_boxes, yolo_confs, yolo_cls_ids, yolo_net); 

        // Step 2: 位姿解算（从YOLO框解算装甲板位姿）
        std::vector<Armors> armors = armorsdetector.detect(frame, yolo_boxes, yolo_confs, yolo_cls_ids);

        // Step 3: 卡尔曼滤波跟踪
        if (!armors.empty()) {
            // 取置信度最高的装甲板
            Armors best_armor = armors[0];

            // 校验位姿有效性
            bool pose_valid = !best_armor.tvec.empty() && cv::checkRange(best_armor.tvec) && 
                              std::isfinite(best_armor.yaw) && best_armor.distance > 0;

            if (pose_valid) {
                // 位姿有效：构造3D位置
                cv::Point3f detect_point(
                    static_cast<float>(best_armor.tvec.at<double>(0)),
                    static_cast<float>(best_armor.tvec.at<double>(1)),
                    static_cast<float>(best_armor.tvec.at<double>(2))
                );
                double detect_yaw = best_armor.yaw;

                // 初始化/更新卡尔曼滤波器
                if (!tracker_inited) {
                    tracker.init(detect_point, detect_yaw);
                    tracker_inited = true;
                    last_valid_position = detect_point;
                    last_valid_yaw = detect_yaw;
                } else {
                    tracker.predict();
                    tracker.update(detect_point, detect_yaw);
                    last_valid_position = detect_point;
                    last_valid_yaw = detect_yaw;
                }

                // 获取预测结果并可视化
                cv::Point3f predict_point = tracker.getPosition();
                double predict_yaw = tracker.getYaw();
                drawTrack(frame, best_armor, predict_point, predict_yaw);
            } else {
                // 位姿无效：仅预测
                std::cerr << "[WARNING] 装甲板位姿无效，仅执行预测" << std::endl;
                if (tracker_inited) {
                    tracker.predict();
                    cv::Point3f predict_point = tracker.getPosition();
                    double predict_yaw = tracker.getYaw();
                    drawTrack(frame, best_armor, predict_point, predict_yaw);
                }
            }
        } else {
            // 未检测到装甲板：仅预测（如果已初始化）
            if (tracker_inited) {
                tracker.predict();
                cv::Point3f predict_point = tracker.getPosition();
                double predict_yaw = tracker.getYaw();
                drawTrack(frame, Armors(), predict_point, predict_yaw);
            }
        }

        // ===================== 可视化 =====================
        cv::imshow("Armor Tracker (YOLO+Kalman)", frame);
        int key = cv::waitKey(30); // 30ms ≈ 30帧/秒
        if (key == 113) { // q键退出
            std::cout << "用户退出程序！" << std::endl;
            break;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
