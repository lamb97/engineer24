//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_EYE_UVC_CAMERA_H
#define CUBOT_EYE_UVC_CAMERA_H

#include <string>
#include <chrono>
#include <thread>
#include <sched.h>
#include <sys/prctl.h>
#include <sys/sysinfo.h>
#include <shared_mutex>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "uvc_camera_data.h"
#include "uvc_camera_param.h"

/**
 * @brief 使用步骤如下：\n
 *       Step1：实例化UvcCamera\n
 *       Step2：自行构造UvcCameraParam或从yaml文件中读取UvcCameraParam\n
 *       Step3：调用SetParam()函数设置相机参数\n
 *       Step4：调用Init()函数初始化相机\n
 *       Step5：调用Open()函数打开相机\n
 *       Step6：调用GetData()函数读取相机数据\n
 *       Step7：调用Close()函数关闭相机\n
 *       Step8：调用Release()函数释放相机资源\n
 */
class UvcCamera
{
public:
    /**
     * @brief 构造函数
     */
    UvcCamera();

    /**
     * @brief 析构函数
     */
    ~UvcCamera();

    /**
     * @brief 拷贝构造函数
     * @param[in] camera 拷贝对象
     * @note 禁用拷贝构造函数
     */
    UvcCamera(const UvcCamera &camera) = delete;

    /**
     * @brief 拷贝赋值运算符
     * @param[in] camera 拷贝对象
     * @return 拷贝赋值结果
     * @note 禁用拷贝赋值运算符
     */
    UvcCamera& operator=(const UvcCamera &camera) = delete;

    /**
     * @brief 移动构造函数
     * @param[in] camera 移动对象
     * @note 禁用移动构造函数
     */
    UvcCamera(UvcCamera &&camera) = delete;

    /**
     * @brief 移动赋值运算符
     * @param[in] camera 移动对象
     * @return 移动赋值结果
     * @note 禁用移动赋值运算符
     */
    UvcCamera& operator=(UvcCamera &&camera) = delete;

    /**
     * @brief 获取相机参数
     * @return 相机参数
     */
    UvcCameraParam GetParam();

    /**
     * @brief 设置相机参数
     * @param[in] param 相机参数
     * @return 设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     * @note 如果相机已经初始化，相机参数将会设置失败
     */
    bool SetParam(const UvcCameraParam &param);

    /**
     * @brief 获取相机的初始化状态
     * @return 相机的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取相机的打开状态
     * @return 相机的打开状态\n
     *         -<em>false</em> 尚未打开\n
     *         -<em>true</em> 已经打开\n
     */
    bool IsOpened();

    /**
     * @brief 获取相机的工作状态
     * @return 相机的工作状态\n
     *         -<em>false</em> 异常状态\n
     *         -<em>true</em> 正常状态\n
     * @note 如果相机打开之后，连续100帧出现读取错误，工作状态变为异常
     */
    bool IsNormal();

    /**
     * @brief 获取相机的初始化时间戳
     * @return 相机的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 获取相机的打开时间戳
     * @return 相机的打开时间戳
     */
    uint64_t GetOpenTimestamp();

    /**
     * @brief 初始化相机
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init();

    /**
     * @brief 释放相机资源
     * @return 释放结果\
     *         -<em>false</em> 释放失败\n
     *         -<em>true</em> 释放成功\n
     * @note Release()和Init()配套使用；相机必须关闭之后才能执行释放操作。
     */
    bool Release();

    /**
     * @brief 打开相机
     * @return 打开结果\n
     *         -<em>false</em> 打开失败\n
     *         -<em>true</em> 打开成功\n
     */
    bool Open();

    /**
     * @brief 关闭相机
     * @return 关闭结果\n
     *         -<em>false</em> 关闭失败\n
     *         -<em>true</em> 关闭成功\n
     */
    bool Close();

    /**
     * @brief 切换相机硬件参数
     * @param hardwareParamIndex 硬件参数索引
     * @return 切换结果\n
     *         -<em>false</em> 切换失败\n
     *         -<em>true</em> 切换成功\n
     * @note 相机的硬件参数必须在初始化之后打开之前进行切换，否则将会切换失败
     */
    bool SwitchHardwareParam(const unsigned int &hardwareParamIndex);

    /**
     * @brief 重置缓存的相机数据帧
     */
    void ResetData();

    /**
     * @brief 获取缓存的相机数据帧
     * @param[out] data 相机数据帧
     */
    void GetData(UvcCameraData *data);

private:
    UvcCameraParam param_;                   ///< 相机参数
    std::atomic<bool> isInitialized_;        ///< 相机初始化状态
    std::atomic<bool> isOpened_;             ///< 相机打开状态
    std::atomic<bool> isNormal_;             ///< 相机工作状态
    std::atomic<uint64_t> initTimestamp_;    ///< 相机初始化时间戳
    std::atomic<uint64_t> openTimestamp_;    ///< 相机打开时间戳
    std::atomic<int> frameWidth_;            ///< 相机数据帧宽度
    std::atomic<int> frameHeight_;           ///< 相机数据帧高度
    cv::VideoCapture videoCapture_;          ///< 相机视频捕捉器
    cv::VideoWriter videoWriter_;            ///< 相机视频录制器
    std::mutex operateMutex_;                ///< 相机操作互斥锁
    UvcCameraData data_;                     ///< 相机缓存数据帧
    mutable std::shared_mutex dataMutex_;    ///< 相机缓存数据帧读写锁
    bool updateDataSwitch_;                  ///< 相机缓存数据帧更新开关
    std::thread updateDataThread_;           ///< 相机缓存数据帧更新线程
    bool recordVideoSwitch_;                 ///< 相机缓存数据帧录制开关
    std::thread recordVideoThread_;          ///< 相机缓存数据帧录制线程

    /**
     * @brief 更新缓存的相机数据帧
     */
    void UpdateData();

    /**
     * @brief 通过离线视频更新缓存的相机数据帧
     */
    void UpdateDataFromOfflineVideo();

    /**
     * @brief 通过在线设备更新缓存的相机数据帧
     */
    void UpdateDataFromOnlineDevice();

    /**
     * @brief 录制缓存的相机数据帧
     */
    void RecordVideo();
};

#endif //CUBOT_EYE_UVC_CAMERA_H