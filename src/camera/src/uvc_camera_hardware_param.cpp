//
// Created by plutoli on 2022/4/5.
//

#include "uvc_camera_hardware_param.h"

// ******************************  UvcCameraHardwareParam类的公有函数  ******************************

// 构造函数
UvcCameraHardwareParam::UvcCameraHardwareParam():
    WorkModes(),
    FrameRate(30),
    Brightness(0),
    ExposureType(EExposureType::Manual),
    ExposureTime(20),
    GainRaw(0),
    Gamma(0),
    Saturation(0),
    Contrast(0),
    Width(1920),
    Height(1080),
    IsSelected(false)
{
}

// 转换相机曝光类型
bool UvcCameraHardwareParam::ConvertToExposureType(const int &input, EExposureType *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换曝光类型
    switch (input)
    {
        case 1:
            *output = EExposureType::Manual;
            break;

        case 3:
            *output = EExposureType::Auto;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}