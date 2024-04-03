//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_EYE_UVC_CAMERA_PARAM_H
#define CUBOT_EYE_UVC_CAMERA_PARAM_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "uvc_camera_runtime_param.h"
#include "uvc_camera_model_param.h"
#include "uvc_camera_hardware_param.h"

/**
 * @brief UVC相机参数
 */
class UvcCameraParam
{
public:
    std::string Key;                                       ///< 相机的标识符
    std::string Name;                                      ///< 相机的驱动文件名
    UvcCameraRuntimeParam RuntimeParam;                    ///< 相机的运行时参数
    UvcCameraModelParam ModelParam;                        ///< 相机的模型参数
    std::vector<UvcCameraHardwareParam> HardwareParams;    ///< 相机的硬件参数集合

    /**
     * @brief 构造函数
     */
    UvcCameraParam();

    /**
     * @brief 析构函数
     */
    ~UvcCameraParam() = default;

    /**
     * @brief 从yaml配置文件中加载相机参数
     * @param[in]  yamlFileName     相机参数配置文件名
     * @param[out] uvcCameraParam   相机参数
     * @return 加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, UvcCameraParam *uvcCameraParam);
};

#endif //CUBOT_EYE_UVC_CAMERA_PARAM_H