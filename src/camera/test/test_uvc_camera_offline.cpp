//
// Created by plutoli on 2022/4/6.
//

#include <string>
#include "uvc_camera.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建相机
    UvcCamera camera;

    // 读取相机参数
    UvcCameraParam cameraParam;
    std::string cameraYaml = "config/basement/aided_rescue_camera_param.yaml";
    if (!UvcCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera.SetParam(cameraParam))
    {
        return -1;
    }

    // 初始化相机
    if (!camera.Init())
    {
        return -1;
    }

    // 打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 记录起始时间戳
    std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
    uint64_t startTimestamp = startTime.time_since_epoch().count();

    // 播放离线视频
    UvcCameraData data;
    while (true)
    {
        // 获取相机数据
        camera.GetData(&data);

        // 播放相机数据
        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("camera", data.Image.cols, data.Image.rows);
        cv::imshow("camera", data.Image);
        cv::waitKey(1);

        // 判断是否播放完毕
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t nowTimestamp = now.time_since_epoch().count();
        if ((nowTimestamp - startTimestamp) > 200000000000)
        {
            break;
        }
    }

    return 0;
}