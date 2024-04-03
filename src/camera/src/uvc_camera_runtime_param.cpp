//
// Created by plutoli on 2022/4/5.
//

#include "uvc_camera_runtime_param.h"

// ******************************  UvcCameraRuntimeParam类的公有函数  ******************************

// 构造函数
UvcCameraRuntimeParam::UvcCameraRuntimeParam():
    IsOnline(true),
    IsRecordVideo(false),
    UpdateDataCpuCore(-1),
    RecordVideoCpuCore(-1),
    RecordVideoFps(20.0),
    OfflineVideoName(),
    RecordVideoPath()
{
}