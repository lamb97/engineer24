//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_EYE_UVC_CAMERA_DATA_H
#define CUBOT_EYE_UVC_CAMERA_DATA_H

#include <opencv2/opencv.hpp>
#include <chrono>

/**
 * @brief UVC相机数据帧
 * @note 基于cv::Mat封装的UVC相机数据帧
 */
class UvcCameraData
{
public:
    /**
     * @brief UVC相机数据帧索引
     */
    uint64_t Index;

    /**
     * @brief UVC相机数据帧的时间戳；单位：纳秒；换算关系：1秒(s)=1000毫秒(ms)=1000,000微秒(us)=1000,000,000纳秒(ns)\n
     * @code
     *  std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
     *  uint64_t timestamp = now.time_since_epoch().count();
     * @endcode
     */
    uint64_t Timestamp;

    /**
     * @brief UVC相机数据帧图像
     */
    cv::Mat Image;

    /**
     * @brief 构造函数
     */
    UvcCameraData();

    /**
     * @brief 析构函数
     */
    ~UvcCameraData() = default;
};

#endif //CUBOT_EYE_UVC_CAMERA_DATA_H