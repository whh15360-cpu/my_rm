 /*#include "Congfig.hpp"
 #include "Struct.hpp"
 #include "PoseSlove.hpp"
 #include <algorithm>
 #include <cmath>
 #include <cfloat>*/




 // 图像预处理
/*cv::Mat ArmorsDetector::preprocessImage(const cv::Mat &img) 
{
    // 高斯模糊和颜色空间转换
    cv::Mat imgGaus, imgHSV;
    cv::GaussianBlur(img, imgGaus, cv::Size(GAUSSIAN_SIZE, GAUSSIAN_SIZE), GAUSSIAN_SIGMA);
    cv::cvtColor(imgGaus, imgHSV, cv::COLOR_BGR2HSV);
     // 颜色阈值分割
     cv::Mat mask1, mask2, maskRED;
     cv::inRange(imgHSV, cv::Scalar(RED_LOWER1_H, RED_LOWER1_S, RED_LOWER1_V),
                 cv::Scalar(RED_UPPER1_H, RED_UPPER1_S, RED_UPPER1_V), mask1);
     cv::inRange(imgHSV, cv::Scalar(RED_LOWER2_H, RED_LOWER2_S, RED_LOWER2_V),
                 cv::Scalar(RED_UPPER2_H, RED_UPPER2_S, RED_UPPER2_V), mask2);
     // 合并两部分红色掩码
     cv::bitwise_or(mask1, mask2, maskRED);

     // 形态学操作
     cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
     cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_KERNEL_SIZE1, MORPH_KERNEL_SIZE1));
     cv::Mat img_N;
     cv::dilate(maskRED, maskRED, kernel1);
     cv::morphologyEx(maskRED, img_N, cv::MORPH_CLOSE, kernel);
     cv::morphologyEx(img_N, img_N, cv::MORPH_OPEN, kernel);
     cv::dilate(img_N, img_N, kernel1);

     return img_N;
 }

 // 调整旋转矩形角度，使其长边接近垂直方向
 void adjustRotatedRect(cv::RotatedRect& rect, const float angle_to_up)
 {
     // 旋转矩形的angle角度范围为[-90, 0)，要转换为[0,90]的范围，方便后续处理
     float angle = rect.angle;
     if( angle < 0 ) 
     {
         angle += 90.0f;
     }

     // 调整角度，使矩形长边接近垂直方向
     if (std::abs(angle - angle_to_up) > 45.0f)
     {
         std::swap(rect.size.width, rect.size.height);
         rect.angle = angle - 90.0f;
     }
 }

 // 灯条检测
 std::vector<Light> ArmorsDetector::detectLights(const cv::Mat &mask) 
 {
     // 寻找轮廓
     std::vector<std::vector<cv::Point>> contours;
     std::vector<cv::Vec4i> hierarchy;
     cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

     // 筛选灯条
     std::vector<Light> lights;
     for (size_t i = 0; i < contours.size(); i++) 
     {
         if (contours[i].size() < 5) continue; // 拟合椭圆至少需要5个点

         double area = cv::contourArea(contours[i]);
         if (area < AREA) continue;

         // 获取最小外接矩形
         // cv::RotatedRect rect = cv::minAreaRect(contours[i]);

         cv::RotatedRect rect;
         try 
         {
             rect = cv::fitEllipse(contours[i]);
         } 
         catch (...) 
         {
             rect = cv::minAreaRect(contours[i]); // 椭圆拟合失败时降级
         }
         // adjustRotatedRect(rect, ANGLE_TO_UP); // 调整角度使其接近垂直

         // 规范化宽高
         float width = rect.size.width;
         float height = rect.size.height;
         if (width > height) cv::swap(width, height);

         // 计算比例和角度
         float ratio = 0.0f; // 【必做】变量显式初始化
         const float eps = 1e-6; // 浮点精度容错值，避免除0
         if (width > eps) {
             ratio = height / width;
         }

         float angle = fabs(rect.angle);

         // 计算轮廓的圆形度（灯条为细长矩形，圆形度接近0；反光点为圆形，圆形度接近1）
         float perimeter = cv::arcLength(contours[i], true);
         float circularity = 0.0f; // 【必做】变量显式初始化
         if (perimeter > eps) {
             circularity = (4 * CV_PI * area) / (perimeter * perimeter);
         }

         bool isNoise = (circularity > 0.8f) && (ratio < 1.8f); // 根据经验值判断是否为噪声
         if (isNoise) continue;
         // 筛选符合比例和角度要求的灯条
         if (ratio > RATIO_MIN && ratio < RATIO_MAX && angle < ANGLE) {
             Light light;
             light.rect = rect;
             light.rat = ratio;
             light.AbsAngle = angle;
             light.center = rect.center;
             //memset(light.vertices, 0, sizeof(light.vertices));
             // 更好的初始化方式
             for (auto& vertex : light.vertices) 
             {
                     vertex = cv::Point2f(0, 0);
             }

             cv::Point2f pts[4];
             light.rect.points(pts);

             for (int k = 0; k < 4; k++) 
             {
                 light.vertices[k] = pts[k];
             }
             lights.push_back(light);
         }
     }
     // 调试：打印检测到的灯条数量
     std::cout << "检测到灯条数量：" << lights.size() << std::endl;

     return lights;
 }

 // 装甲板配对
 std::vector<Armors> ArmorsDetector::matchArmors(const std::vector<Light> &lights) 
 {
     std::vector<Armors> armors;
     if (lights.size() < 2) return armors;

     // 两两配对灯条
     for (size_t i = 0; i < lights.size(); i++) 
     {
         for (size_t j = i + 1; j < lights.size(); j++) 
         {
             const Light &bar1 = lights[i];
             const Light &bar2 = lights[j];

             const Light &leftBar = bar1.center.x < bar2.center.x ? bar1 : bar2;
             const Light &rightBar = bar1.center.x < bar2.center.x ? bar2 : bar1;

             float distance = cv::norm(leftBar.center - rightBar.center);
             float heightAvg = (leftBar.rect.size.height + rightBar.rect.size.height) / 2;

             const float eps = 1e-6; // 浮点精度容错值，避免除0
             if (heightAvg < eps) continue; // 避免除0

             // 恢复垂直偏差筛选（关键！）
              float yDiff = std::fabs(leftBar.center.y - rightBar.center.y);
              if (yDiff > heightAvg * 0.5f) continue; // 垂直偏差≤50%

             // 距离筛选
             //if (distance < heightAvg * DISTANCE_MIN || distance > heightAvg * DISTANCE_MAX) 
                 //continue; 

             //float angle = fabs(atan2(rightBar.center.y - leftBar.center.y , rightBar.center.x - leftBar.center.x) * 180 / CV_PI);
             //if (angle > ARMOR_ANGLE) continue;

             // float heightDiff = fabs(leftBar.rect.size.height - rightBar.rect.size.height) / std::max(leftBar.rect.size.height, rightBar.rect.size.height);
             // if (heightDiff > HEIGHT_DIFF) continue;

             // float angleDiff = fabs(leftBar.AbsAngle - rightBar.AbsAngle);
             // if (angleDiff > ANGLE_DIFF) continue;

             //float armorRatio = distance / heightAvg;
             // if (armorRatio < ARMOR_RATIO_MIN || armorRatio > ARMOR_RATIO_MAX) continue;
          

             if (distance < heightAvg * DISTANCE_MIN || distance > heightAvg * DISTANCE_MAX) 
         {
             continue;
         }

         float angle = std::abs(std::atan2(rightBar.center.y - leftBar.center.y, rightBar.center.x - leftBar.center.x) * 180.0f / CV_PI);
            if (angle > ARMOR_ANGLE)
         {
             continue;
         }
        
        float heightDiff = std::abs(leftBar.rect.size.height - rightBar.rect.size.height) / std::max(leftBar.rect.size.height, rightBar.rect.size.height);
         if (heightDiff > HEIGHT_DIFF) 
         {
             continue;
         }
        
         float angleDiff = std::abs(leftBar.AbsAngle - rightBar.AbsAngle);
         if (angleDiff > ANGLE_DIFF) 
         {
             continue;
         }
        
         float armorRatio = distance / heightAvg;
         if (armorRatio < ARMOR_RATIO_MIN  || armorRatio > ARMOR_RATIO_MAX) 
         {
             continue;
       }
             Armors armor;
             armor.left = leftBar;
             armor.right = rightBar;
             armor.center = (leftBar.center + rightBar.center) * 0.5f;

             cv::Point2f armorPts[4];
             armorPts[0] = leftBar.vertices[1].y < leftBar.vertices[3].y ? leftBar.vertices[1] : leftBar.vertices[3];
             armorPts[1] = rightBar.vertices[0].y < rightBar.vertices[2].y ? rightBar.vertices[0] : rightBar.vertices[2];
             armorPts[2] = rightBar.vertices[0].y > rightBar.vertices[2].y ? rightBar.vertices[0] : rightBar.vertices[2];
             armorPts[3] = leftBar.vertices[1].y > leftBar.vertices[3].y ? leftBar.vertices[1] : leftBar.vertices[3];

             armor.corners.resize(4);  // 
             std::copy(std::begin(armorPts), std::end(armorPts), std::begin(armor.corners));

             std::vector<cv::Point2f> pts_vec(armorPts, armorPts + 4);
             armor.boundingRect = cv::minAreaRect(pts_vec);
            
             armors.push_back(armor);
         }
     }
     // 调试：打印装甲板数量
     std::cout << "检测到装甲板数量：" << armors.size() << std::endl;

     return armors;
 }*/