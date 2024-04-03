//
// Created by plutoli on 2021/8/14.
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
    std::string cameraYaml = "config/basement/aided_pick_camera_param.yaml";
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

    // 初始化上一帧图像的时间戳
    uint64_t previousTimestamp = 0;

    // 初始化播放的视频帧数
    uint64_t displayedFrameNumber = 0;

    // 记录起始时间戳
    std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
    uint64_t startTimestamp = startTime.time_since_epoch().count();

    // 播放在线视频
    UvcCameraData data;
    while (true)
    {
        // 记录开始计算时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 获取相机数据
        camera.GetData(&data);
        if (data.Timestamp > previousTimestamp)
        {
            previousTimestamp = data.Timestamp;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 输出分隔符
        std::cout << std::endl;
        std::cout << "**************************************************" << std::endl;

        // 输出播放的视频帧数和相机数据索引
        displayedFrameNumber++;
        std::cout << "displayedFrameNumber: " << displayedFrameNumber << std::endl;
        std::cout << "cameraDataIndex: " << data.Index << std::endl;

        // 播放视频
        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("camera", data.Image.cols, data.Image.rows);
        cv::imshow("camera", data.Image);
        cv::waitKey(3);

        // 读取并输出相机状态
        std::cout << "Key: " << camera.GetParam().Key << std::endl;
        std::cout << "IsNormal: " << camera.IsNormal() << std::endl;

        // 输出分隔符
        std::cout << "**************************************************" << std::endl;

        // 记录停止计算时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 输出时间消耗
        std::cout << "Total time span: " << (endTimestamp - beginTimestamp) / 1000 << "us" << std::endl;

        // 判断是否播放完毕
        if ((endTimestamp - startTimestamp) > 100000000000)
        {
            break;
        }
    }

    return 0;
}