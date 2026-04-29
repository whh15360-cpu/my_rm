#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>


#include "Config.hpp"
#include "Yolo.hpp"
#include "PoseSolve.hpp"
#include "Draw.hpp"


// 封装进 Node 类
class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() : Node("armor_detector_node") {
        // 放置初始化逻辑
        auto& config = Config::getInstance();
        std::string calib_path = "/home/hh/ros2_ws/src/armor_old/config/calib_result.yml";
        if (!config.loadFromFile(calib_path)) {
            RCLCPP_ERROR(this->get_logger(), "无法打开标定文件");
            return;
        }

        std::string model_path = "/home/hh/ros2_ws/src/armor_old/config/best.onnx";
        std::string label_path = "/home/hh/ros2_ws/src/armor_old/config/labels.txt";
        yolo_solver_ = std::make_unique<YOLOSolver>(model_path, label_path);
        armor_detector_ = std::make_unique<ArmorsDetector>();

        //cap_.open(0);
        //if (!cap_.isOpened()) {
            //RCLCPP_ERROR(this->get_logger(), "Capture open failed!");
            //return;
        //}
        
        std::string video_path = "/home/hh/ros2_ws/src/armor_old/config/new_red9.mp4"; // 视频路径
        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path.c_str());
            return;
         }

        // 创建定时器替代 while
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 约 30 FPS
            std::bind(&ArmorDetectorNode::run_inference, this)
        );
    }

private:
    void run_inference() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        
        auto yolo_results = yolo_solver_->detect(frame);
        
        std::vector<cv::Rect> boxes;
        std::vector<float> confs;
        std::vector<int> class_ids;
        for (auto& res : yolo_results) {
            boxes.push_back(res.bbox);
            confs.push_back(res.confidence);
            class_ids.push_back(res.class_id);
        }

        auto armors = armor_detector_->detect(frame, boxes, confs, class_ids);

        if (!armors.empty()) {
    const auto& target = armors[0];
    
        cv::circle(frame, target.center, 5, cv::Scalar(0, 255, 0), -1);   //中心绿点
    
        std::string txt = "Dist: " + std::to_string(target.distance).substr(0, 4) + "m";
        cv::Point2f text_org = target.center + cv::Point2f(10, 10);
    
        cv::putText(frame, txt, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 4);  // 白色粗描边 
    
        cv::putText(frame, txt, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 0), 2);     // 黑色细文字 
}

        cv::imshow("Debug", frame);
        //cv::waitKey(1);
        
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            RCLCPP_INFO(this->get_logger(), "停止运行节点");
            rclcpp::shutdown(); // 通知 ROS2 停止所有节点运行
        }
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<YOLOSolver> yolo_solver_;
    std::unique_ptr<ArmorsDetector> armor_detector_;
};

// 2. 简化的 main 函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  //ros2的入口
    auto node = std::make_shared<ArmorDetectorNode>();   //进入继承的类ArmorDetectorNode
    rclcpp::spin(node);  //spin 会使程序进入阻塞循环，持续监听订阅的消息（如相机原始画面）、处理定时器回调以及响应服务请求
    rclcpp::shutdown(); //函数结束前调用，确保关闭，释放资源并向系统告知节点下线
    return 0;
}




