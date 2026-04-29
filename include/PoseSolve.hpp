#ifndef POSE_SOLVE_H
#define POSE_SOLVE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "Struct.hpp"


struct Armors {
    //int class_id;
    Light left;
    Light right;
    std::vector<cv::Point2f> corners;
    cv::RotatedRect boundingRect;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Point3f pos;
    cv::Point2f center;
    float distance = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f; 
    float roll = 0.0f;
    //float confidence;
    //float kf_z;     // 滤波后
    
};

class ArmorsDetector {
public:
    
    ArmorsDetector(); 
    ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    
    std::vector<Armors> detect(const cv::Mat& frame, 
                               const std::vector<cv::Rect>& yoloBoxes, 
                               const std::vector<float>& confidences, 
                               const std::vector<int>& classIds);

    void draw(cv::Mat &img, const std::vector<Armors> &armors);

private:
    bool solvePose(Armors& armor);\
    cv::Mat cameraMatrix_; 
    cv::Mat distCoeffs_;

    // 硬编码，为了防止config.hpp反复出错，采用此方法
    float light_min_area = 20.0f;
    float light_max_angle = 45.0f;
    float light_max_angle_diff = 6.0f;
    float armor_ratio_min = 1.0f;
    float armor_ratio_max = 3.5f;
};

#endif
