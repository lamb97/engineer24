//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_EYE_UVC_CAMERA_MODEL_PARAM_H
#define CUBOT_EYE_UVC_CAMERA_MODEL_PARAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief UVC相机的模型参数；包括内参矩阵、外参矩阵、畸变系数向量
 * @note 相机模型原理资料如下：\n
 *       https://blog.csdn.net/weixin_39928787/article/details/111117833 \n
 *       https://blog.csdn.net/weixin_43206570/article/details/84797361 \n
 *       https://blog.csdn.net/reasonyuanrobot/article/details/86614381 \n
 */
class UvcCameraModelParam
{
public:
    cv::Mat CvIntrinsics;                               ///< 相机OpenCV格式的内参矩阵
    cv::Mat CvExtrinsics;                               ///< 相机OpenCV格式的外参矩阵
    cv::Mat CvDistortions;                              ///< 相机OpenCV格式的畸变系数向量
    Eigen::Matrix3d EigenIntrinsics;                    ///< 相机Eigen格式的内参矩阵
    Eigen::Isometry3d EigenExtrinsics;                  ///< 相机Eigen格式的外参矩阵
    Eigen::Matrix<double, 5, 1> EigenDistortions;       ///< 相机Eigen格式的畸变系数向量

    /**
    * @brief 构造函数
    */
    UvcCameraModelParam();

    /**
     * @brief 析构函数
     */
    ~UvcCameraModelParam() = default;
};

#endif //CUBOT_EYE_UVC_CAMERA_MODEL_PARAM_H