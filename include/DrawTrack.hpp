#ifndef DRAWTRACK_HPP
#define DRAWTRACK_HPP

#include <opencv2/opencv.hpp>
#include "Struct.hpp"
#include "Config.hpp"
#include "PoseSlove.hpp"
#include "ExtendedKalman.hpp"

void drawTrack(cv::Mat& img, 
            const Armors& detected_armor, 
            const cv::Point3f& predict_position, 
            double predict_yaw,
            const cv::Mat& last_valid_rvec,
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs);

#endif // DRAWTRACK_HPP
