#include <iostream>
#include "serial_port_param.h"
#include "serial_port.h"

// 串口数据接收缓冲区
unsigned char receivedBuffer[1000];

// 串口数据接收回调函数
void HandleDataReceived(const unsigned int &bytesToRead, void* userData)
{
    // 读取一行数据
    auto *serialPort = (SerialPort *)userData;
    unsigned char size = serialPort->ReadLine(receivedBuffer);
    receivedBuffer[size] = '\0';

    // 记录接收到的时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    uint64_t timestamp = now.time_since_epoch().count();

    // 显示读取到的数据
    std::cout << "[" << timestamp << "] - data = " << (char*)receivedBuffer << std::endl;
}

// Linux命令行列出所有的串口设备：dmesg | grep ttyS*
int main(int, char* [])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建串口
    SerialPort serialPort;

    // 加载串口参数
    SerialPortParam serialPortParam;
    std::string yameFile = "config/2022-03-20/serial_port_param.yaml";
    SerialPortParam::LoadFromYamlFile(yameFile, &serialPortParam);

    // 设置串口参数
    serialPort.SetParam(serialPortParam);

    // 注册串口的数据接收回调函数
    serialPort.RegisterDataReceivedHandler(HandleDataReceived, &serialPort);

    // 初始化串口
    serialPort.Init();

    // 打开串口
    serialPort.Open();

    // 循环发送数据
    for (unsigned int i = 0; i < 3000; ++i)
    {
        // 发送数据
        std::string data = "This is a output test: " + std::to_string(i) + "\n";
        serialPort.Write(data.size(), (const unsigned char *)data.c_str());

        // usleep()的输入参数单位为微秒(us)，换算关系：1s=1000ms=1000000us。休眠2ms
        usleep(2000);
    }

    // 关闭串口
    serialPort.Close();

    // 释放串口资源
    serialPort.Release();

    return 0;
}