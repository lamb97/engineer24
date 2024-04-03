//
// Created by plutoli on 2022/4/5.
//

#include "uvc_camera.h"

// ******************************  UvcCamera类的公有函数  ******************************

// 构造函数
UvcCamera::UvcCamera():
    param_(),
    isInitialized_(false),
    isOpened_(false),
    isNormal_(false),
    initTimestamp_(0),
    openTimestamp_(0),
    frameWidth_(1920),
    frameHeight_(1080),
    videoCapture_(),
    videoWriter_(),
    operateMutex_(),
    data_(),
    dataMutex_(),
    updateDataSwitch_(false),
    updateDataThread_(),
    recordVideoSwitch_(false),
    recordVideoThread_()
{
    ResetData();
}

// 析构函数
UvcCamera::~UvcCamera()
{
    // 关闭相机
    if (IsOpened())
    {
        Close();
    }

    // 释放相机资源
    if (IsInitialized())
    {
        Release();
    }
}

// 获取相机参数
UvcCameraParam UvcCamera::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置相机参数
bool UvcCamera::SetParam(const UvcCameraParam &param)
{
    // 锁定相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断相机是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - UvcCameraParam was set failure because Camera has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录相机参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - UvcCameraParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回设置结果
    return true;
}

// 获取相机的初始化状态
bool UvcCamera::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取相机的打开状态
bool UvcCamera::IsOpened()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isOpened_;
}

// 获取相机的工作状态
bool UvcCamera::IsNormal()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isNormal_;
}

// 获取相机的初始化时间戳
uint64_t UvcCamera::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 获取相机的打开时间戳
uint64_t UvcCamera::GetOpenTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return openTimestamp_;
}

// 初始化相机
bool UvcCamera::Init()
{
    // 锁定相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断相机是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - Camera can not be initialized repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 根据相机的运行状态进行初始化
    if (param_.RuntimeParam.IsOnline)
    {
        // 如果相机需要录像，判断录像文件存储路径是否合法
        if (param_.RuntimeParam.IsRecordVideo)
        {
            // 判断录像文件存储路径是否存在
            if (::access(param_.RuntimeParam.RecordVideoPath.c_str(), R_OK) == -1)
            {
                if (::mkdir(param_.RuntimeParam.RecordVideoPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
                {
                    log = "[" + param_.Key + "] - Camera was initialized failure because RuntimeParam's RecordVideoPath is invalid";
                    logger.Save(ELogType::Error, log);
                    log = LOG_END;
                    logger.Save(ELogType::Info, log);
                    return false;
                }
            }
        }
    }
    else
    {
        // 判断离线视频文件是否存在
        if (::access(param_.RuntimeParam.OfflineVideoName.c_str(), F_OK) == -1)
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because RuntimeParam's OfflineVideo does not exist";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }

        // 判断离线视频文件是否可读
        if (::access(param_.RuntimeParam.OfflineVideoName.c_str(), R_OK) == -1)
        {
            log = "[" + param_.Key + "] - Camera was initialized failure because RuntimeParam's OfflineVideo can not be read";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }
    }

    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放相机资源
bool UvcCamera::Release()
{
    // 锁定相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断相机是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - Camera was released failure because it has been opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断相机是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - Camera can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回释放结果
    return true;
}

// 打开相机
bool UvcCamera::Open()
{
    // 锁定相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断相机是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - Camera was opened failure because it has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断相机是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - Camera can not be opened repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 根据相机的运行状态打开相机
    if (param_.RuntimeParam.IsOnline)
    {
        // 搜索选中的相机硬件参数
        int hardwareParamIndex = -1;
        for (int i = 0; i < param_.HardwareParams.size(); ++i)
        {
            if (param_.HardwareParams[i].IsSelected)
            {
                hardwareParamIndex = i;
                break;
            }
        }

        // 判断是否搜索到选中的相机硬件参数
        if (hardwareParamIndex == -1)
        {
            log = "[" + param_.Key + "] - Camera was opened failure because there is no selected hardware param";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }

        // 打开相机的视频捕捉器，设置相机属性
        if (videoCapture_.open(param_.Name))
        {
            // 记录日志信息
            log = "[" + param_.Key + "] - Camera was connected successful";
            logger.Save(ELogType::Info, log);

            // 设置相机的数据帧格式属性
            if (videoCapture_.set(cv::CAP_PROP_FOURCC,
                                  cv::VideoWriter::fourcc('M', 'J', 'P', 'G')))
            {
                log = "[" + param_.Key + "] - Camera's Format property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Format property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧帧率属性
            if (videoCapture_.set(cv::CAP_PROP_FPS,
                                  param_.HardwareParams[hardwareParamIndex].FrameRate))
            {
                log = "[" + param_.Key + "] - Camera's FrameRate property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because FrameRate property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧亮度属性
            if (videoCapture_.set(cv::CAP_PROP_BRIGHTNESS,
                                  param_.HardwareParams[hardwareParamIndex].Brightness))
            {
                log = "[" + param_.Key + "] - Camera's Brightness property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Brightness property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的自动曝光属性
            auto exposureType = static_cast<double>(param_.HardwareParams[hardwareParamIndex].ExposureType);
            if (videoCapture_.set(cv::CAP_PROP_AUTO_EXPOSURE, exposureType))
            {
                log = "[" + param_.Key + "] - Camera's ExposureType property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because ExposureType property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧曝光时间属性
            if (videoCapture_.set(cv::CAP_PROP_EXPOSURE,
                                  param_.HardwareParams[hardwareParamIndex].ExposureTime))
            {
                log = "[" + param_.Key + "] - Camera's ExposureTime property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because ExposureTime property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧增益值属性
            if (videoCapture_.set(cv::CAP_PROP_GAIN,
                                  param_.HardwareParams[hardwareParamIndex].GainRaw))
            {
                log = "[" + param_.Key + "] - Camera's GainRaw property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because GainRaw property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧伽马值属性
            if (videoCapture_.set(cv::CAP_PROP_GAMMA,
                                  param_.HardwareParams[hardwareParamIndex].Gamma))
            {
                log = "[" + param_.Key + "] - Camera's Gamma property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Gamma property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的饱和度属性
            if (videoCapture_.set(cv::CAP_PROP_SATURATION,
                                  param_.HardwareParams[hardwareParamIndex].Saturation))
            {
                log = "[" + param_.Key + "] - Camera's Saturation property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Saturation property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的对比度属性
            if (videoCapture_.set(cv::CAP_PROP_CONTRAST,
                                  param_.HardwareParams[hardwareParamIndex].Contrast))
            {
                log = "[" + param_.Key + "] - Camera's Contrast property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Contrast property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧宽度属性
            if (videoCapture_.set(cv::CAP_PROP_FRAME_WIDTH,
                                  param_.HardwareParams[hardwareParamIndex].Width))
            {
                frameWidth_ = static_cast<int>(param_.HardwareParams[hardwareParamIndex].Width);
                log = "[" + param_.Key + "] - Camera's Width property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Width property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }

            // 设置相机的数据帧高度属性
            if (videoCapture_.set(cv::CAP_PROP_FRAME_HEIGHT,
                                  param_.HardwareParams[hardwareParamIndex].Height))
            {
                frameHeight_ = static_cast<int>(param_.HardwareParams[hardwareParamIndex].Height);
                log = "[" + param_.Key + "] - Camera's Height property was set successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because Height property can not be set";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was opened failure because it can't be connected";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }

        // 如果需要录像，则打开相机视频录制器
        if (param_.RuntimeParam.IsRecordVideo)
        {
            // 获取当前日期时间字符串
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::localtime(&now_time);
            std::stringstream stream;
            stream << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");
            std::string dateTimeString = stream.str();

            // 创建录像文件名称和录像文件的绝对路径
            std::string recordFileName = "[" + param_.Key + "] - RawVideo_" + dateTimeString + ".avi";
            std::string fullPath = param_.RuntimeParam.RecordVideoPath + recordFileName;

            // 打开相机视频录制器
            if (videoWriter_.open(fullPath,
                                  cv::VideoWriter::fourcc('M','J','P','G'),
                                  param_.RuntimeParam.RecordVideoFps,
                                  cv::Size(frameWidth_, frameHeight_),
                                  true))
            {
                log = "[" + param_.Key + "] - Camera was started recording successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - Camera was opened failure because it can't be started recording";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }
        }
    }
    else
    {
        // 打开视频捕捉器
        if(videoCapture_.open(param_.RuntimeParam.OfflineVideoName))
        {
            // 记录日志信息
            log = "[" + param_.Key + "] - OfflineVideo has been opened successful";
            logger.Save(ELogType::Info, log);

            // 读取离线视频文件数据帧的高度和宽度
            frameWidth_ = static_cast<int>(videoCapture_.get(cv::CAP_PROP_FRAME_WIDTH));
            frameHeight_ = static_cast<int>(videoCapture_.get(cv::CAP_PROP_FRAME_HEIGHT));
        }
        else
        {
            log = "[" + param_.Key + "] - Camera was opened failure because OfflineVideo can't be opened";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }
    }

    // 启动相机缓存数据帧更新线程
    updateDataSwitch_ = true;
    updateDataThread_ = std::thread(&UvcCamera::UpdateData, this);
    log = "[" + param_.Key + "] - Camera's UpdateDataThread was started successful";
    logger.Save(ELogType::Info, log);

    // 启动相机缓存数据帧录制线程
    if (param_.RuntimeParam.IsOnline && param_.RuntimeParam.IsRecordVideo)
    {
        recordVideoSwitch_ = true;
        recordVideoThread_ = std::thread(&UvcCamera::RecordVideo, this);
        log = "[" + param_.Key + "] - Camera's RecordVideoThread was started successful";
        logger.Save(ELogType::Info, log);
    }

    // 设置打开时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    openTimestamp_ = now.time_since_epoch().count();

    // 设置打开标志
    isOpened_ = true;

    // 设置工作状态
    isNormal_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was opened successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回打开结果
    return true;
}

// 关闭相机
bool UvcCamera::Close()
{
    // 锁定相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断相机是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - Camera can not be closed repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 停止相机缓存数据帧录制线程
    if (param_.RuntimeParam.IsOnline && param_.RuntimeParam.IsRecordVideo)
    {
        recordVideoSwitch_ = false;
        if (recordVideoThread_.joinable())
        {
            recordVideoThread_.join();
        }
        log = "[" + param_.Key + "] - Camera's RecordVideoThread was stopped successful";
        logger.Save(ELogType::Info, log);
    }

    // 停止相机缓存数据帧更新线程
    updateDataSwitch_ = false;
    if (updateDataThread_.joinable())
    {
        updateDataThread_.join();
    }
    log = "[" + param_.Key + "] - Camera's UpdateDataThread was stopped successful";
    logger.Save(ELogType::Info, log);

    // 释放相机视频录制器
    if (videoWriter_.isOpened())
    {
        videoWriter_.release();
    }

    // 释放相机视频捕捉器
    if (videoCapture_.isOpened())
    {
        videoCapture_.release();
    }

    // 重置打开时间戳
    openTimestamp_ = 0;

    // 重置打开状态
    isOpened_ = false;

    // 重置工作状态
    isNormal_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - Camera was closed successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回关闭结果
    return true;
}

// 切换相机硬件参数
bool UvcCamera::SwitchHardwareParam(const unsigned int &hardwareParamIndex)
{
    // 锁定相机的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断相机是否处于在线运行状态
    if (!param_.RuntimeParam.IsOnline)
    {
        log = "[" + param_.Key + "] - UvcCameraHardwareParam was switched failure because camera is not in Online mode";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断相机是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - UvcCameraHardwareParam was switched failure because camera has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断相机是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - UvcCameraHardwareParam was switched failure because camera has been opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 更新相机硬件参数属性
    if (hardwareParamIndex < param_.HardwareParams.size())
    {
        // 重置所有相机硬件参数的选择状态
        for (unsigned int i = 0; i < param_.HardwareParams.size(); ++i)
        {
            param_.HardwareParams[i].IsSelected = false;
        }

        // 设置要切换的硬件参数的选择状态
        param_.HardwareParams[hardwareParamIndex].IsSelected = true;
    }
    else
    {
        log = "[" + param_.Key + "] - UvcCameraHardwareParam was switched failure because hardwareParamIndex is invalid";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录日志信息
    log = "[" + param_.Key + "] - UvcCameraHardwareParam was switched successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回切换结果
    return true;
}

// 重置缓存的相机数据帧
void UvcCamera::ResetData()
{
    // 锁定相机数据缓冲区
    std::unique_lock<std::shared_mutex> uniqueLock(dataMutex_);

    // 重置相机数据帧的索引
    data_.Index = 0;

    // 重置相机数据帧的时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    data_.Timestamp = now.time_since_epoch().count();

    // 重置相机数据帧的图像
    data_.Image = cv::Mat(frameHeight_,
                          frameWidth_,
                          CV_8UC3,
                          cv::Scalar(0, 0, 255));
    cv::putText(data_.Image,
                "The cached data has been reset",
                cv::Point(frameWidth_ / 2 - 200, frameHeight_ / 2),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 0, 0),
                2);
}

// 获取缓存的相机数据帧
void UvcCamera::GetData(UvcCameraData *data)
{
    std::shared_lock<std::shared_mutex> sharedLock(dataMutex_);
    data->Index = data_.Index;
    data->Timestamp = data_.Timestamp;
    data_.Image.copyTo(data->Image);
}

// ******************************  UvcCamera类的私有函数  ******************************

// 更新缓存的相机数据帧
void UvcCamera::UpdateData()
{
    // 修改线程名称
    std::string threadName = "update_camera_data";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.RuntimeParam.UpdateDataCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 更新缓存的相机数据帧
    if (param_.RuntimeParam.IsOnline)
    {
        UpdateDataFromOnlineDevice();
    }
    else
    {
        UpdateDataFromOfflineVideo();
    }
}

// 通过离线视频更新缓存的相机数据帧
void UvcCamera::UpdateDataFromOfflineVideo()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 读取离线视频文件的帧数
    int frameCount = static_cast<int>(videoCapture_.get(cv::CAP_PROP_FRAME_COUNT));

    // 读取离线视频文件的帧率，计算相邻数据帧之间的事件延迟(微秒)
    double fps = videoCapture_.get(cv::CAP_PROP_FPS);
    int frameDelay = static_cast<int>(1000000.0 / fps);

    // 循环读取离线视频图像
    cv::Mat image;
    while (updateDataSwitch_ && videoCapture_.read(image))
    {
        // 获取时间戳
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t timestamp = now.time_since_epoch().count();

        // 更新缓存的相机数据帧
        dataMutex_.lock();
        data_.Index++;
        data_.Timestamp = timestamp;
        image.copyTo(data_.Image);
        dataMutex_.unlock();

        // 线程延时，保证离线视频图像读取速度正常
        std::this_thread::sleep_for(std::chrono::microseconds(frameDelay));

        // 获取当前帧图像的位置，判断当前帧图像是否为最后一帧图像
        // TODO 对于某些离线视频文件，使用capture.get(cv::CAP_PROP_FRAME_COUNT)函数读取到的frameCount为0；\n
        //      无法通过framePosition判断是否播放到最后一帧图像，必须重新打开离线视频文件。
        int framePosition = static_cast<int>(videoCapture_.get(cv::CAP_PROP_POS_FRAMES));
        if ((frameCount > 0) && (framePosition == frameCount))
        {
            // 重置缓存的相机数据帧
            ResetData();

            // 重置当前帧图像的位置
            videoCapture_.set(cv::CAP_PROP_POS_FRAMES, 0);

            // 记录日志信息
            log = "[" + param_.Key + "] - OfflineVideo's image position has been reset";
            logger.Save(ELogType::Info, log);
        }
    }

    // 记录日志信息
    if (updateDataSwitch_)
    {
        log = "[" + param_.Key + "] - OfflineVideo playback was completed because it's invalid FrameCount property";
        logger.Save(ELogType::Error, log);
    }
    else
    {
        log = "[" + param_.Key + "] - OfflineVideo playback was completed gracefully";
        logger.Save(ELogType::Info, log);
    }
}

// 通过在线设备更新缓存的相机数据帧
void UvcCamera::UpdateDataFromOnlineDevice()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 初始化异常状态计数器
    unsigned int abnormalCounter = 0;

    // 循环读取原始数据帧
    cv::Mat image;
    while (updateDataSwitch_)
    {
        // 获取原始数据帧
        if (!videoCapture_.read(image))
        {
            // 判断并修改相机的工作状态
            // 注意：在实际使用时，可以根据实际需要修改判断条件
            abnormalCounter++;
            if (abnormalCounter >= 100)
            {
                isNormal_ = false;
            }

            // 记录日志信息
            log = "[" + param_.Key + "] - Raw frame was read failure";
            logger.Save(ELogType::Error, log);

            // 延迟10毫秒
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            continue;
        }

        // 重置异常状态计数器
        abnormalCounter = 0;

        // 获取当前时间戳
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t timestamp = now.time_since_epoch().count();

        // 更新缓存的相机数据帧
        dataMutex_.lock();
        data_.Index++;
        data_.Timestamp = timestamp;
        image.copyTo(data_.Image);
        dataMutex_.unlock();
    }
}

// 录制缓存的相机数据帧
void UvcCamera::RecordVideo()
{
    // 修改线程名称
    std::string threadName = "record_camera_data";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.RuntimeParam.RecordVideoCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 初始化临时图像
    cv::Mat image;

    // 计算采样周期
    auto samplePeriod = static_cast<uint64_t>(1000000000.0 / param_.RuntimeParam.RecordVideoFps);

    // 录制相机数据帧
    while (recordVideoSwitch_)
    {
        // 获取起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 复制相机数据帧中的图像
        dataMutex_.lock();
        data_.Image.copyTo(image);
        dataMutex_.unlock();

        // 保存相机数据帧中的图像
        videoWriter_.write(image);

        // 获取截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 计算相机数据帧的保存时间
        uint64_t writeSpan = endTimestamp - beginTimestamp;

        // 线程延时，确保录制视频的帧率正常
        if (samplePeriod > writeSpan)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(samplePeriod - writeSpan));
        }
    }
}