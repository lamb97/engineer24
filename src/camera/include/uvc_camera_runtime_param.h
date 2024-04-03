//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_EYE_UVC_CAMERA_RUNTIME_PARAM_H
#define CUBOT_EYE_UVC_CAMERA_RUNTIME_PARAM_H

#include <string>

/**
 * @brief UVC相机运行时参数
 */
class UvcCameraRuntimeParam
{
public:
    bool IsOnline;                      ///< 是否在线运行
    bool IsRecordVideo;                 ///< 是否录制视频
    int UpdateDataCpuCore;              ///< 更新数据帧任务的CPU内核编号；默认为-1
    int RecordVideoCpuCore;             ///< 录制视频流任务的CPU内核编号；默认为-1
    double RecordVideoFps;              ///< 录制视频的帧率；默认为20.0
    std::string OfflineVideoName;       ///< 离线视频文件名称
    std::string RecordVideoPath;        ///< 录像文件存储路径

    /**
     * @brief 构造函数
     */
    UvcCameraRuntimeParam();

    /**
     * @brief 析构函数
     */
    ~UvcCameraRuntimeParam() = default;
};

#endif //CUBOT_EYE_UVC_CAMERA_RUNTIME_PARAM_H