//
// Created by lamb97 on 23-11-27.
//

#ifndef ENGINEER_RECOGNIZE_H
#define ENGINEER_RECOGNIZE_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif //ENGINEER_RECOGNIZE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <numeric>

//求两向量夹角
double angle_of_vector(double v1[], double v2[]) {
    const double pi = 3.1415;
    double vector_prod = v1[0] * v2[0] + v1[1] * v2[1];
    double length_prod = sqrt(pow(v1[0], 2) + pow(v1[1], 2)) * sqrt(pow(v2[0], 2) + pow(v2[1], 2));
    double cos = vector_prod / (length_prod + 1e-6);
    return (acos(cos) / pi) * 180;
}

//获取contour中的center
std::vector<double> get_center_of_contours(std::vector<std::vector<cv::Point>>& contours) {
    double sum_x = 0;
    double sum_y = 0;
    int num = 0;
    for (const auto& contour : contours) {
        for (const auto& point : contour)
        {
            sum_x += point.x;
            sum_y += point.y;
            num++;
        }
    }
    std::vector<double> center = {sum_x / num, sum_y / num};
    return center;
}
//获取contour距离center最远的点
cv::Point get_farest_point(const std::vector<cv::Point>& contour, const cv::Point& center) {
    double farest_length_pow = 0;
    cv::Point point_goal;
    for (const auto& point : contour) {
        double length_pow = std::pow(point.x - center.x, 2) + std::pow(point.y - center.y, 2);
        if (length_pow > farest_length_pow) {
            farest_length_pow = length_pow;
            point_goal = point;
        }
    }
    return point_goal;
}
//获取两点距离
double len_2point(const cv::Point& p1, const cv::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
//对contours进行大小滤波
std::vector<std::vector<cv::Point>> filt_contours_size(const std::vector<std::vector<cv::Point>>& contours, double contour_area_min = 100, double contour_area_max = 500)
{
    std::vector<std::vector<cv::Point>> filted_small_contours;
    for (const auto& contour : contours) {
        double contour_area = cv::contourArea(contour);
        if (contour_area > contour_area_min && contour_area < contour_area_max) {
            filted_small_contours.push_back(contour);
        }
    }
    return filted_small_contours;
}

// 滤波函数
std::pair<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>>
filt_contours_pos(const std::vector<std::vector<cv::Point>>& contours,
                  double rate_range = 0.2,
                  double ang_range = 20.0) {

    // 循环检查每种排列组合
    for(size_t i = 0; i < contours.size() - 3; i++) {
        for(size_t j = i+1; j < contours.size() - 2; j++) {
            for(size_t k = j+1; k < contours.size() - 1; k++) {
                for(size_t l = k+1; l < contours.size(); l++) {
                    std::vector<std::vector<cv::Point>> contours_permutation = {contours[i], contours[j], contours[k], contours[l]};
                    std::vector<cv::Point> top_points_permutation;

//                    // 找到四个色块的顶点
//                    cv::Point2f center = get_center_of_contours(contours_permutation);
//                    for (auto& contour : contours_permutation) {
//                        cv::Point top_point = get_farest_point(contour, center);
//                        top_points_permutation.push_back(top_point);
//                    }

                    // 接下来进行以前代码中的剩余判定

                }
            }
        }
    }

    // 如果没有找到满足条件的，返回空的结果
    return std::make_pair(std::vector<std::vector<cv::Point>>(), std::vector<cv::Point>());
}



//std::pair<std::vector<cv::Point2f>, cv::Mat> get_rgb_pos(const cv::Mat& img) {
//    // HSV 颜色范围
//    cv::Scalar hsv_min = cv::Scalar(132, 82, 82);
//    cv::Scalar hsv_max = cv::Scalar(201, 167, 170);
//
//    // 转换到 HSV 空间并二值化
//    cv::Mat img_hsv, img_mask;
//    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
//    cv::inRange(img_hsv, hsv_min, hsv_max, img_mask);
//
//    // 形态学闭操作
//    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
//    cv::Mat img_close;
//    cv::morphologyEx(img_mask, img_close, cv::MORPH_CLOSE, kernel);
//
//    // 转换到可视化图像
//    cv::Mat img_show;
//    //cv::cvtColor(img_close.copy(), img_show, cv::COLOR_GRAY2BGR);
//
//    // 寻找轮廓
//    std::vector<std::vector<cv::Point>> contours;
//    cv::findContours(img_close, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//
//    // 绘制原始轮廓
//    cv::drawContours(img_show, contours, -1, cv::Scalar(0, 0, 255), 1);
//
//    // 大小过滤
//    std::vector<std::vector<cv::Point>> size_filted_contours;
//    filt_contours_size(contours, 100, 500);
//
//    // 绘制大小过滤后的轮廓
//    cv::drawContours(img_show, size_filted_contours, -1, cv::Scalar(255, 0, 0), 1);
//
//    // 位置过滤
//    auto [pos_filted_contours, centers_permutation] = filt_contours_pos(size_filted_contours, 0.1f);
//
//    if (pos_filted_contours.empty()) {
//        return { std::vector<cv::Point2f>(), img_show };  // or appropriate error signaling
//    }
//
//    // 标记过滤后的轮廓
//    cv::drawContours(img_show, pos_filted_contours, -1, cv::Scalar(0, 255, 0), 1);
//
//    // Link the points with lines
//    for (size_t i = 0; i < centers_permutation.size(); ++i) {
//        cv::line(img_show, centers_permutation[i], centers_permutation[(i+1)%centers_permutation.size()], cv::Scalar(0, 255, 0), 1);
//    }
//
//    // Get the furthest points
//    std::vector<cv::Point2f> point_goals;
//    cv::Point2f center = get_center_of_contours(pos_filted_contours);
//    for (const auto& contour : pos_filted_contours) {
//        cv::Point2f point_goal = get_farest_point(contour, center);
//        point_goals.push_back(point_goal);
//        cv::drawMarker(img_show, point_goal, cv::Scalar(0, 255, 0), cv::MARKER_CROSS);
//    }
//    cv::drawMarker(img_show, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS);
//
//    return { point_goals, img_show };
//}


//模板匹配，滤除离群点,找到方框包围matches
std::pair<cv::Point2i, cv::Point2i> sift_match_filt_outliers(
        const cv::Mat& img,
        const cv::Mat& template_img,
        float filt_coefficient = 1.5f,
        float rect_expand_coefficient = 0.1f)
{
    // 初始化SIFT检测器
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> kp_template, kp_img;
    cv::Mat des_template, des_img;
    cv::Point weight;

    // 检测和计算keypoints和descriptors
    sift->detectAndCompute(template_img, cv::noArray(), kp_template, des_template);
    sift->detectAndCompute(img, cv::noArray(), kp_img, des_img);

    // 开始匹配
    cv::BFMatcher bf(cv::NORM_L2, true);  // 交叉检查
    std::vector<cv::DMatch> matches;
    bf.match(des_template, des_img, matches);

    // 计算匹配点的平均位置
    cv::Point2f center(0, 0);
    for (const auto& match : matches) {
        cv::Point2f train_pt = kp_img[match.trainIdx].pt;
        center += train_pt;
    }
    center.x /= matches.size();
    center.y /= matches.size();

    // 计算距离并过滤离群点
    std::vector<float> distances(matches.size());
    float distances_sum = 0;
    for (size_t i = 0; i < matches.size(); ++i) {
        cv::Point2f train_pt = kp_img[matches[i].trainIdx].pt;
        distances[i] = len_2point(train_pt, center);
        distances_sum += distances[i];
    }
    float distances_average = distances_sum / matches.size();

    std::vector<cv::DMatch> result;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (distances[i] < distances_average * filt_coefficient) {
            result.push_back(matches[i]);
        }
    }

    // 计算最大的匹配点框的位置
    float up = kp_img[result[0].trainIdx].pt.y;
    float down = up;
    float left = kp_img[result[0].trainIdx].pt.x;
    float right = left;

    for (const auto& match : result) {
        cv::Point2f train_pt = kp_img[match.trainIdx].pt;
        up = std::min(up, train_pt.y);
        down = std::max(down, train_pt.y);
        left = std::min(left, train_pt.x);
        right = std::max(right, train_pt.x);
    }

    // 扩大框的大小
    up = std::max(0.0f, up - (down - up) * rect_expand_coefficient);
    left = std::max(0.0f, left - (right - left) * rect_expand_coefficient);
    down = std::min(static_cast<float>(img.rows - 1), down + (down - up) * rect_expand_coefficient);
    right = std::min(static_cast<float>(img.cols - 1), right + (right - left) * rect_expand_coefficient);

    return {cv::Point2i(static_cast<int>(left), static_cast<int>(up)), cv::Point2i(static_cast<int>(right), static_cast<int>(down))};
}
// Function to calculate the Euclidean distance between two points
double len_2point(cv::Point2f p1, cv::Point2f p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

//// 若点a大于点b,即点a在点b顺时针方向,返回false,否则返回true
//bool PointCmp(const Point2f &a, const Point2f &b, const Point2f &center) {
//
//    if (a.x >= 0 && b.x < 0)
//        return true;
//    if (a.x == 0 && b.x == 0)
//        return a.y > b.y;
//
//    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
//    if (det < 0)
//        return true;
//    if (det > 0)
//        return false;
//
//    double d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
//    double d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
//    return d1 > d2;
//}
//
//void ClockwiseSortPoints(vector<Point2f> &vPoints) {
//    // 计算重心
//    Point2f center;
//    double X = 0, Y = 0;
//    for (unsigned int i = 0; i < vPoints.size(); i++) {
//        X += vPoints[i].x;
//        Y += vPoints[i].y;
//    }
//    center.x = static_cast<int>(X / vPoints.size());
//    center.y = static_cast<int>(Y / vPoints.size());
//
//    // 冒泡排序
//    for (unsigned int i = 0; i < vPoints.size() - 1; i++) {
//        for (unsigned int j = 0; j < vPoints.size() - i - 1; j++) {
//            if (PointCmp(vPoints[j], vPoints[j + 1], center)) {
//                swap(vPoints[j], vPoints[j + 1]);
//            }
//        }
//    }
//}