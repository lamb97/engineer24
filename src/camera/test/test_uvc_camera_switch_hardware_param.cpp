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

    // 创建相机并初始化相机数据
    UvcCamera camera;
    UvcCameraData cameraData;

    // 读取相机参数
    UvcCameraParam cameraParam;
    std::string cameraYaml = "config/basement/aided_collect_camera_param.yaml";
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

    // 切换第一组硬件参数
    if (!camera.SwitchHardwareParam(0))
    {
        return -1;
    }

    // 打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 记录第一组硬件参数时间戳
    std::chrono::time_point<std::chrono::steady_clock> firstTime = std::chrono::steady_clock::now();
    uint64_t firstTimestamp = firstTime.time_since_epoch().count();

    // 播放第一组硬件参数的相机视频
    while (true)
    {
        // 获取相机数据
        camera.GetData(&cameraData);

        // 播放视频
        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("camera", cameraData.Image.cols, cameraData.Image.rows);
        cv::imshow("camera", cameraData.Image);
        cv::waitKey(1);

        // 判断是否播放完毕
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t nowTimestamp = now.time_since_epoch().count();
        if ((nowTimestamp - firstTimestamp) > 30000000000)
        {
            break;
        }
    }

    // 关闭相机
    camera.Close();

    // 切换第二组参数
    if (!camera.SwitchHardwareParam(1))
    {
        return -1;
    }

    // 打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 记录第二组硬件参数时间戳
    std::chrono::time_point<std::chrono::steady_clock> secondTime = std::chrono::steady_clock::now();
    uint64_t secondTimestamp = secondTime.time_since_epoch().count();

    // 播放第二组硬件参数的相机视频
    while (true)
    {
        // 获取相机数据
        camera.GetData(&cameraData);

        // 播放视频
        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("camera", cameraData.Image.cols, cameraData.Image.rows);
        cv::imshow("camera", cameraData.Image);
        cv::waitKey(1);

        // 判断是否播放完毕
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t nowTimestamp = now.time_since_epoch().count();
        if ((nowTimestamp - secondTimestamp) > 30000000000)
        {
            break;
        }
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    return 0;
}