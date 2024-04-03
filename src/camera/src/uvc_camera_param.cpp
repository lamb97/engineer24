//
// Created by plutoli on 2022/4/5.
//

#include "uvc_camera_param.h"


// ******************************  UvcCameraParam类的公有函数  ******************************

// 构造函数
UvcCameraParam::UvcCameraParam():
    Key(),
    Name(),
    RuntimeParam(),
    ModelParam(),
    HardwareParams()
{
}

// 从yaml配置文件中加载相机参数
bool UvcCameraParam::LoadFromYamlFile(const std::string &yamlFileName, UvcCameraParam *uvcCameraParam)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断yaml配置文件是否存在
    if (::access(yamlFileName.c_str(), F_OK) == -1)
    {
        log = "UvcCameraParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "UvcCameraParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "UvcCameraParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        uvcCameraParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + uvcCameraParam->Key + "] - UvcCameraParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + uvcCameraParam->Key + "] - UvcCameraParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Name参数
    if ((!fileStorage["Name"].isNone()) && (fileStorage["Name"].isString()))
    {
        uvcCameraParam->Name = static_cast<std::string>(fileStorage["Name"]);
        log = "[" + uvcCameraParam->Key + "] - UvcCameraParam's Name was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + uvcCameraParam->Key + "] - UvcCameraParam's Name was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取相机的运行时参数
    cv::FileNode runtimeParamNode = fileStorage["RuntimeParam"];
    if (!runtimeParamNode.empty())
    {
        // 读取IsOnline参数
        if ((!runtimeParamNode["IsOnline"].isNone()) && (runtimeParamNode["IsOnline"].isInt()))
        {
            uvcCameraParam->RuntimeParam.IsOnline = static_cast<int>(runtimeParamNode["IsOnline"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's IsOnline was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's IsOnline was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取IsRecordVideo参数
        if ((!runtimeParamNode["IsRecordVideo"].isNone()) && (runtimeParamNode["IsRecordVideo"].isInt()))
        {
            uvcCameraParam->RuntimeParam.IsRecordVideo = static_cast<int>(runtimeParamNode["IsRecordVideo"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's IsRecordVideo was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's IsRecordVideo was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取UpdateDataCpuCore参数
        if ((!runtimeParamNode["UpdateDataCpuCore"].isNone()) && (runtimeParamNode["UpdateDataCpuCore"].isInt()))
        {
            uvcCameraParam->RuntimeParam.UpdateDataCpuCore = static_cast<int>(runtimeParamNode["UpdateDataCpuCore"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's UpdateDataCpuCore was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's UpdateDataCpuCore was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取RecordVideoCpuCore参数
        if ((!runtimeParamNode["RecordVideoCpuCore"].isNone()) && (runtimeParamNode["RecordVideoCpuCore"].isInt()))
        {
            uvcCameraParam->RuntimeParam.RecordVideoCpuCore = static_cast<int>(runtimeParamNode["RecordVideoCpuCore"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's RecordVideoCpuCore was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's RecordVideoCpuCore was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取RecordVideoFps参数
        if ((!runtimeParamNode["RecordVideoFps"].isNone()) && (runtimeParamNode["RecordVideoFps"].isReal()))
        {
            uvcCameraParam->RuntimeParam.RecordVideoFps = static_cast<double>(runtimeParamNode["RecordVideoFps"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's RecordVideoFps was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's RecordVideoFps was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取OfflineVideoName参数
        if ((!runtimeParamNode["OfflineVideoName"].isNone()) && (runtimeParamNode["OfflineVideoName"].isString()))
        {
            uvcCameraParam->RuntimeParam.OfflineVideoName = static_cast<std::string>(runtimeParamNode["OfflineVideoName"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's OfflineVideoName was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's OfflineVideoName was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取RecordVideoPath参数
        if ((!runtimeParamNode["RecordVideoPath"].isNone()) && (runtimeParamNode["RecordVideoPath"].isString()))
        {
            uvcCameraParam->RuntimeParam.RecordVideoPath = static_cast<std::string>(runtimeParamNode["RecordVideoPath"]);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's RecordVideoPath was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam's RecordVideoPath was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + uvcCameraParam->Key + "] - UvcCameraRuntimeParam was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取相机的模型参数
    cv::FileNode modelParamNode = fileStorage["ModelParam"];
    if (!modelParamNode.empty())
    {
        // 读取CvIntrinsics参数
        if ((!modelParamNode["CvIntrinsics"].isNone()) && (modelParamNode["CvIntrinsics"].isMap()))
        {
            modelParamNode["CvIntrinsics"] >> uvcCameraParam->ModelParam.CvIntrinsics;
            uvcCameraParam->ModelParam.EigenIntrinsics.resize(uvcCameraParam->ModelParam.CvIntrinsics.rows,
                                                              uvcCameraParam->ModelParam.CvIntrinsics.cols);
            cv::cv2eigen(uvcCameraParam->ModelParam.CvIntrinsics, uvcCameraParam->ModelParam.EigenIntrinsics);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam's CvIntrinsics was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam's CvIntrinsics was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取CvExtrinsics参数
        if ((!modelParamNode["CvExtrinsics"].isNone()) && (modelParamNode["CvExtrinsics"].isMap()))
        {
            modelParamNode["CvExtrinsics"] >> uvcCameraParam->ModelParam.CvExtrinsics;
            cv::cv2eigen(uvcCameraParam->ModelParam.CvExtrinsics,
                         uvcCameraParam->ModelParam.EigenExtrinsics.matrix());
            log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam's CvExtrinsics was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam's CvExtrinsics was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取CvDistortions参数
        if ((!modelParamNode["CvDistortions"].isNone()) && (modelParamNode["CvDistortions"].isMap()))
        {
            modelParamNode["CvDistortions"] >> uvcCameraParam->ModelParam.CvDistortions;
            cv::cv2eigen(uvcCameraParam->ModelParam.CvDistortions,
                         uvcCameraParam->ModelParam.EigenDistortions);
            log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam's CvDistortions was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam's CvDistortions was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + uvcCameraParam->Key + "] - UvcCameraModelParam was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取相机的硬件参数
    cv::FileNode hardwareParamsNode = fileStorage["HardwareParams"];
    if (!hardwareParamsNode.empty())
    {
        unsigned int hardwareParamIndex = 0;
        cv::FileNodeIterator hardwareParamIterator = hardwareParamsNode.begin();
        while (hardwareParamIterator != hardwareParamsNode.end())
        {
            // 创建硬件参数
            UvcCameraHardwareParam hardwareParam;

            // 读取WorkModes参数
            cv::FileNode workModesNode = (*hardwareParamIterator)["WorkModes"];
            if (!workModesNode.empty())
            {
                unsigned int workModeIndex = 0;
                cv::FileNodeIterator workModeIterator = workModesNode.begin();
                while (workModeIterator != workModesNode.end())
                {
                    if ((*workModeIterator).isInt())
                    {
                        EWorkMode workMode;
                        if (SystemConfigurator::ConvertToWorkMode(static_cast<int>(*workModeIterator),
                                                                  &workMode))
                        {
                            hardwareParam.WorkModes.emplace_back(workMode);
                            log = "[" + uvcCameraParam->Key + "] - "\
                                  "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]."\
                                  "WorkModes[" + std::to_string(workModeIndex) + "] was loaded successful";
                            logger.Save(ELogType::Info, log);
                        }
                        else
                        {
                            log = "[" + uvcCameraParam->Key + "] - "\
                                  "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]."\
                                  "WorkModes[" + std::to_string(workModeIndex) + "] was converted failure";
                            logger.Save(ELogType::Error, log);
                        }
                    }
                    else
                    {
                        log = "[" + uvcCameraParam->Key + "] - "\
                              "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]."\
                              "WorkModes[" + std::to_string(workModeIndex) + "] was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 工作模式索引和迭代器累加
                    workModeIndex++;
                    workModeIterator++;
                }
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "WorkModes was loaded failure because it is empty";
                logger.Save(ELogType::Error, log);
            }

            // 读取FrameRate参数
            if ((!(*hardwareParamIterator)["FrameRate"].isNone()) && ((*hardwareParamIterator)["FrameRate"].isReal()))
            {
                hardwareParam.FrameRate = static_cast<double>((*hardwareParamIterator)["FrameRate"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "FrameRate was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "FrameRate was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Brightness参数
            if ((!(*hardwareParamIterator)["Brightness"].isNone()) && ((*hardwareParamIterator)["Brightness"].isReal()))
            {
                hardwareParam.Brightness = static_cast<double>((*hardwareParamIterator)["Brightness"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Brightness was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Brightness was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ExposureType参数
            if ((!(*hardwareParamIterator)["ExposureType"].isNone()) &&
                ((*hardwareParamIterator)["ExposureType"].isInt()))
            {
                EExposureType exposureType;
                if (UvcCameraHardwareParam::ConvertToExposureType(static_cast<int>((*hardwareParamIterator)["ExposureType"]),
                                                                  &exposureType))
                {
                    hardwareParam.ExposureType = exposureType;
                    log = "[" + uvcCameraParam->Key + "] - "\
                          "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                          "ExposureType was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + uvcCameraParam->Key + "] - "\
                          "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                          "ExposureType was converted failure";
                    logger.Save(ELogType::Error, log);
                }
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "ExposureType was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ExposureTime参数
            if ((!(*hardwareParamIterator)["ExposureTime"].isNone()) &&
                ((*hardwareParamIterator)["ExposureTime"].isReal()))
            {
                hardwareParam.ExposureTime = static_cast<double>((*hardwareParamIterator)["ExposureTime"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "ExposureTime was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "ExposureTime was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取GainRaw参数
            if ((!(*hardwareParamIterator)["GainRaw"].isNone()) && ((*hardwareParamIterator)["GainRaw"].isReal()))
            {
                hardwareParam.GainRaw = static_cast<double>((*hardwareParamIterator)["GainRaw"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "GainRaw was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "GainRaw was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Gamma参数
            if ((!(*hardwareParamIterator)["Gamma"].isNone()) && ((*hardwareParamIterator)["Gamma"].isReal()))
            {
                hardwareParam.Gamma = static_cast<double>((*hardwareParamIterator)["Gamma"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Gamma was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Gamma was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Saturation参数
            if ((!(*hardwareParamIterator)["Saturation"].isNone()) &&
                ((*hardwareParamIterator)["Saturation"].isReal()))
            {
                hardwareParam.Saturation = static_cast<double>((*hardwareParamIterator)["Saturation"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Saturation was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Saturation was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Contrast参数
            if ((!(*hardwareParamIterator)["Contrast"].isNone()) &&
                ((*hardwareParamIterator)["Contrast"].isReal()))
            {
                hardwareParam.Contrast = static_cast<double>((*hardwareParamIterator)["Contrast"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Contrast was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Contrast was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Width参数
            if ((!(*hardwareParamIterator)["Width"].isNone()) && ((*hardwareParamIterator)["Width"].isInt()))
            {
                hardwareParam.Width = static_cast<double>((*hardwareParamIterator)["Width"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Width was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Width was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Height参数
            if ((!(*hardwareParamIterator)["Height"].isNone()) && ((*hardwareParamIterator)["Height"].isInt()))
            {
                hardwareParam.Height = static_cast<double>((*hardwareParamIterator)["Height"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Height was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Height was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取IsSelected参数
            if ((!(*hardwareParamIterator)["IsSelected"].isNone()) && ((*hardwareParamIterator)["IsSelected"].isInt()))
            {
                hardwareParam.IsSelected = static_cast<int>((*hardwareParamIterator)["IsSelected"]);
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsSelected was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + uvcCameraParam->Key + "] - "\
                      "UvcCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsSelected was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 保存硬件参数
            uvcCameraParam->HardwareParams.emplace_back(hardwareParam);

            // 硬件参数索引和迭代器累加
            hardwareParamIndex++;
            hardwareParamIterator++;
        }
    }
    else
    {
        log = "[" + uvcCameraParam->Key + "] - UvcCameraHardwareParams was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + uvcCameraParam->Key + "] - UvcCameraParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}