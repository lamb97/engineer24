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

    // 打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 重置缓存的相机数据帧
    camera.ResetData();

    // 初始化相机数据
    UvcCameraData data;

    // 播放相机视频
    while (camera.IsNormal())
    {
        // 获取相机数据
        camera.GetData(&data);

        // 播放相机视频
        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("camera", data.Image.cols, data.Image.rows);
        cv::imshow("camera", data.Image);
        cv::waitKey(1);
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    // 重新初始化相机
    if (!camera.Init())
    {
        return -1;
    }

    // 重新打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 播放相机视频
    while (camera.IsNormal())
    {
        // 获取相机数据
        camera.GetData(&data);

        // 播放视频
        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("camera", data.Image.cols, data.Image.rows);
        cv::imshow("camera", data.Image);
        cv::waitKey(1);
    }

    return 0;
}