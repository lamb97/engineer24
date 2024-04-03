//
// Created by plutoli on 2021/12/30.
//

#include "easy_logger.h"

int main(int argc, char *argv[])
{
    // 创建日志记录器参数
    EasyLoggerParam param;
    param.LogPrefix = "train";

    // 设置并初始化日志记录器参数
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.SetParam(param);
    bool result = logger.Init();

    // 保存是指信息
    for (unsigned int i = 0; i < 100; ++i)
    {
        logger.Save(ELogType::Info, "Test log info");
        logger.Save(ELogType::Warn, "Test log warn");
        logger.Save(ELogType::Error, "Test log error");
        sleep(1);
    }

    return 0;
}