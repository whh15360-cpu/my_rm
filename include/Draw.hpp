#ifndef DRAWTRACK_HPP
#define DRAWTRACK_HPP

#include <opencv2/opencv.hpp>
#include "Struct.hpp"
#include "PoseSolve.hpp"

void drawTrack(cv::Mat& img, 
               const Armors& detected_armor, 
               const cv::Point3f& predict_position, 
               double predict_yaw,
               const cv::Mat& last_valid_rvec);
          
#endif
