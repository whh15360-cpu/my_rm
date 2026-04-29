#ifndef STRUCT_H
#define STRUCT_H

#include <opencv2/opencv.hpp>

struct Light {
    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f vertices[4];
    float length;         // 长度
    float width;          // 宽度
    float angle;          // 倾斜角度
    

    // 构造函数列表初始化
    Light() : length(0), width(0), angle(0), center(0, 0) {
        for (auto& v : vertices) {
            v = cv::Point2f(0, 0);
        }
    }

    // 额外添加一个带参构造函数，方便在 Armors.cpp 中直接创建
    explicit Light(cv::RotatedRect r) {
        rect = r;
        center = r.center;
        angle = r.angle;
        // 长边为 length，短边为 width
        length = std::max(r.size.width, r.size.height);
        width = std::min(r.size.width, r.size.height);
        r.points(vertices); 
    }
};

#endif
