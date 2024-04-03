//
// Created by plutoli on 2022/4/5.
//

#include "uvc_camera_model_param.h"

// ******************************  UvcCameraModelParam类的公有函数  ******************************

// 构造函数
UvcCameraModelParam::UvcCameraModelParam():
    CvIntrinsics(),
    CvExtrinsics(),
    CvDistortions(),
    EigenIntrinsics(),
    EigenExtrinsics(),
    EigenDistortions()
{
}