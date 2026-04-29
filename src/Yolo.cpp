#include "Yolo.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>


YOLOSolver::YOLOSolver(const std::string& model_path, const std::string& labels_path) {
    labels_ = loadLabels(labels_path);
    if (labels_.empty()) throw std::runtime_error("无法加载标签文件");

    net_ = cv::dnn::readNetFromONNX(model_path);    //读入 best.onnx 模型，对图片做前向推理
    
    // 先使用CPU 推理，进行调试
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

YOLOSolver::~YOLOSolver() {}

// 加载 labels.txt，明确0和1分别代表什么
std::vector<std::string> YOLOSolver::loadLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) labels.push_back(line);
    }
    return labels;
}

// 预处理，匹配训练时的 imgsz: 320
cv::Mat YOLOSolver::preprocessImage(const cv::Mat& img) {
    cv::Mat blob;
    // 使用 320x320，swapRB=true
    cv::dnn::blobFromImage(img, blob, 1.0/255.0, cv::Size(320, 320), cv::Scalar(0,0,0), true, false);
    return blob;
}

/*std::vector<DetectionResult> YOLOSolver::parseOutput(const cv::Mat& output, const cv::Size& img_size) {
    std::vector<DetectionResult> results;
    
    
    // YOLOv8 输出是 [1, 4+类别数, 2100] (因为 imgsz=320)
    // 处理维度
    cv::Mat raw_data;
    if (output.dims == 3) {
        raw_data = cv::Mat(output.size[1], output.size[2], CV_32F, (float*)output.data);
    } else {
        raw_data = output;
    }
    
    // 转置，使每一行代表一个检测框 [2100, 4+类别数]
    cv::Mat output_mat = raw_data.t(); 

    int rows = output_mat.rows;       // 2100
    int dimensions = output_mat.cols;  // 4 + num_classes
    int num_classes = dimensions - 4;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> bboxes;

    // 计算缩放比例（从 320 还原到原图）
    float scale_x = (float)img_size.width / 320.0f;
    float scale_y = (float)img_size.height / 320.0f;

    for (int i = 0; i < rows; ++i) {
        float* data = output_mat.ptr<float>(i);
        float* classes_scores = data + 4;

        // 获取该框中得分最高的类别
        cv::Mat scores(1, num_classes, CV_32F, classes_scores);
        cv::Point class_id_node;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_node);

        if (max_class_score > 0.45f) { // 阈值参考训练时的 conf 参数
            float cx = data[0];
            float cy = data[1];
            float w = data[2];
            float h = data[3];

            int left = cvRound((cx - w / 2.0f) * scale_x);
            int top = cvRound((cy - h / 2.0f) * scale_y);
            int width = cvRound(w * scale_x);
            int height = cvRound(h * scale_y);

            class_ids.push_back(class_id_node.x);
            confidences.push_back((float)max_class_score);
            bboxes.push_back(cv::Rect(left, top, width, height));
        }
    }

    // NMS 过滤
    std::vector<int> indices;
    cv::dnn::NMSBoxes(bboxes, confidences, 0.45f, 0.45f, indices);

    std::vector<DetectionResult> final_results;
    for (int idx : indices) {
        DetectionResult res;
        res.class_id = class_ids[idx];
        res.class_name = (res.class_id < labels_.size()) ? labels_[res.class_id] : "Unknown";
        res.confidence = confidences[idx];
        res.bbox = bboxes[idx];
        final_results.push_back(res);
    }
    return final_results;
}

std::vector<DetectionResult> YOLOSolver::detect(const cv::Mat& img) {
    if (img.empty()) return {};
    cv::Mat blob = preprocessImage(img);
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    return parseOutput(outputs[0], img.size());
}*/


// 解析模型输出
std::vector<DetectionResult> YOLOSolver::detect(const cv::Mat& output, const cv::Size& img_size) {
    std::vector<DetectionResult> results;
    
    // YOLOv8 输出是 [1, 4 + classes, 2100]
    cv::Mat raw_data = (output.dims == 3) ? 
                       cv::Mat(output.size[1], output.size[2], CV_32F, (float*)output.data) : output;
    cv::Mat output_mat = raw_data.t(); // 转置为 [2100, 4 + classes]

    float scale_x = (float)img_size.width / INPUT_WIDTH;
    float scale_y = (float)img_size.height / INPUT_HEIGHT;

    std::vector<int> class_ids;
    std::vector<float> confs;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < output_mat.rows; i++) {
        float* data = output_mat.ptr<float>(i);
        float* scores_ptr = data + 4;
        
        cv::Mat scores(1, labels_.size(), CV_32F, scores_ptr);
        cv::Point class_id_point;
        double max_score;
        cv::minMaxLoc(scores, 0, &max_score, 0, &class_id_point);

        if (max_score > CONF_THRESHOLD) {
            float cx = data[0], cy = data[1], w = data[2], h = data[3];
            int left = cvRound((cx - w/2) * scale_x);
            int top = cvRound((cy - h/2) * scale_y);
            int width = cvRound(w * scale_x);
            int height = cvRound(h * scale_y);

            boxes.push_back(cv::Rect(left, top, width, height));
            confs.push_back((float)max_score);
            class_ids.push_back(class_id_point.x);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, CONF_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices) {
        DetectionResult res;
        res.bbox = boxes[idx];
        res.confidence = confs[idx];
        res.class_id = class_ids[idx];
        res.class_name = labels_[res.class_id];
        results.push_back(res);
    }
    return results;
    }

    

//  接收单个 img 的逻辑 (Main.cpp 调用的入口)
std::vector<DetectionResult> YOLOSolver::detect(const cv::Mat& img) {
    if (img.empty()) return {};

    //  预处理
    cv::Mat blob = preprocessImage(img);
    
    //  推理
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    //  调用上面的“解析”重载函数
    return detect(outputs[0], img.size());
}


