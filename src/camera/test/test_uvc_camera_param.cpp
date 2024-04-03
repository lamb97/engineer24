//
// Created by plutoli on 2021/8/14.
//

#include "uvc_camera_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 从yaml文件中加载相机参数
    UvcCameraParam param;
    UvcCameraParam::LoadFromYamlFile("config/basement/aided_collect_camera_param.yaml",
                                     &param);

    return 0;
}