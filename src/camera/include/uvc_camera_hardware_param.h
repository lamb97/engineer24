//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_EYE_UVC_CAMERA_HARDWARE_PARAM_H
#define CUBOT_EYE_UVC_CAMERA_HARDWARE_PARAM_H

#include <string>
#include <vector>
#include "system_configurator.h"

/**
 * @brief 相机曝光类型
 */
enum class EExposureType
{
    Manual = 1,    ///< 手动曝光
    Auto = 3       ///< 自动曝光
};

/**
 * @brief UVC相机硬件参数
 */
class UvcCameraHardwareParam
{
public:
    std::vector<EWorkMode> WorkModes;   ///< 相机硬件参数适配的工作模式集合
    double FrameRate;                   ///< 相机帧率
    double Brightness;                  ///< 相机亮度
    EExposureType ExposureType;         ///< 相机曝光类型
    double ExposureTime;                ///< 相机曝光时间
    double GainRaw;                     ///< 相机增益值
    double Gamma;                       ///< 相机伽马值
    double Saturation;                  ///< 相机饱和度
    double Contrast;                    ///< 相机对比度
    double Width;                       ///< 相机图像宽度
    double Height;                      ///< 相机图像高度
    bool IsSelected;                    ///< 相机硬件参数的选择状态

    /**
     * @brief 构造函数
     */
    UvcCameraHardwareParam();

    /**
     * @brief 析构函数
     */
    ~UvcCameraHardwareParam() = default;

    /**
    * @brief 转换相机曝光类型
    * @param[in]   input   输入的相机曝光类型数值
    * @param[out]  output  转换得到的相机曝光类型
    * @return 转换结果\n
    *         -<em>false</em> 转换失败\n
    *         -<em>true</em> 转换成功\n
    * @note 相机曝光类型的取值范围为1/3，输入数据不在此范围内，则转换失败\n
    *         -<em>1</em> Manual\n
    *         -<em>3</em> Auto\n
    */
    static bool ConvertToExposureType(const int &input, EExposureType *output);
};

#endif //CUBOT_EYE_UVC_CAMERA_HARDWARE_PARAM_H