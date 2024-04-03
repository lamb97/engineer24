//
// Created by plutoli on 2021/7/19.
//

#include "serial_port_param.h"

// ******************************  SerialPortParam类的公有函数  ******************************

// 构造函数
SerialPortParam::SerialPortParam():
    Key("SerialPort"),
    Name("/dev/ttyUSB0"),

    BaudRate(EBaudRate::BR115200),
    DataBits(EDataBits::Eight),
    StopBits(EStopBits::One),
    Parity(EParity::None),
    FlowControl(EFlowControl::None),
    ReadFromDriverCpuCore(-1),
    WriteToDriverCpuCore(-1),
    ReadBufferSize(1024),
    WriteBufferSize(1024),
    Separators()
{
}

// 转换波特率
bool SerialPortParam::ConvertToBaudRate(const int &input, EBaudRate *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换波特率
    switch (input)
    {
        case 50:
            *output = EBaudRate::BR50;
            break;

        case 75:
            *output = EBaudRate::BR75;
            break;

        case 110:
            *output = EBaudRate::BR110;
            break;

        case 134:
            *output = EBaudRate::BR134;
            break;

        case 150:
            *output = EBaudRate::BR150;
            break;

        case 200:
            *output = EBaudRate::BR200;
            break;

        case 300:
            *output = EBaudRate::BR300;
            break;

        case 600:
            *output = EBaudRate::BR600;
            break;

        case 1200:
            *output = EBaudRate::BR1200;
            break;

        case 1800:
            *output = EBaudRate::BR1800;
            break;

        case 2400:
            *output = EBaudRate::BR2400;
            break;

        case 4800:
            *output = EBaudRate::BR4800;
            break;

        case 9600:
            *output = EBaudRate::BR9600;
            break;

        case 19200:
            *output = EBaudRate::BR19200;
            break;

        case 38400:
            *output = EBaudRate::BR38400;
            break;

        case 57600:
            *output = EBaudRate::BR57600;
            break;

        case 115200:
            *output = EBaudRate::BR115200;
            break;

        case 230400:
            *output = EBaudRate::BR230400;
            break;

        case 460800:
            *output = EBaudRate::BR460800;
            break;

        case 500000:
            *output = EBaudRate::BR500000;
            break;

        case 576000:
            *output = EBaudRate::BR576000;
            break;

        case 921600:
            *output = EBaudRate::BR921600;
            break;

        case 1000000:
            *output = EBaudRate::BR1000000;
            break;

        case 1152000:
            *output = EBaudRate::BR1152000;
            break;

        case 1500000:
            *output = EBaudRate::BR1500000;
            break;

        case 2000000:
            *output = EBaudRate::BR2000000;
            break;

        case 2500000:
            *output = EBaudRate::BR2500000;
            break;

        case 3000000:
            *output = EBaudRate::BR3000000;
            break;

        case 3500000:
            *output = EBaudRate::BR3500000;
            break;

        case 4000000:
            *output = EBaudRate::BR4000000;
            break;

        default:
            *output = EBaudRate::BR0;
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换数据位
bool SerialPortParam::ConvertToDataBits(const int &input, EDataBits *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换数据位
    switch (input)
    {
        case 5:
            *output = EDataBits::Five;
            break;

        case 6:
            *output = EDataBits::Six;
            break;

        case 7:
            *output = EDataBits::Seven;
            break;

        case 8:
            *output = EDataBits::Eight;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换停止位
bool SerialPortParam::ConvertToStopBits(const int &input, EStopBits *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换停止位
    switch (input)
    {
        case 1:
            *output = EStopBits::One;
            break;

        case 2:
            *output = EStopBits::Two;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换校验位
bool SerialPortParam::ConvertToParity(const int &input, EParity *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换校验位
    switch (input)
    {
        case 0:
            *output = EParity::None;
            break;

        case 1:
            *output = EParity::Odd;
            break;

        case 2:
            *output = EParity::Even;
            break;

        case 3:
            *output = EParity::Mark;
            break;

        case 4:
            *output = EParity::Space;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 转换流控
bool SerialPortParam::ConvertToFlowControl(const int &input, EFlowControl *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换流控
    switch (input)
    {
        case 0:
            *output = EFlowControl::None;
            break;

        case 1:
            *output = EFlowControl::Software;
            break;

        case 2:
            *output = EFlowControl::Hardware;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 从yaml配置文件中加载串口参数
bool SerialPortParam::LoadFromYamlFile(const std::string &yamlFileName, SerialPortParam *serialPortParam)
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
        log = "SerialPortParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "SerialPortParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "SerialPortParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        serialPortParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + serialPortParam->Key + "] - SerialPortParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Name参数
    if ((!fileStorage["Name"].isNone()) && (fileStorage["Name"].isString()))
    {
        serialPortParam->Name = static_cast<std::string>(fileStorage["Name"]);
        log = "[" + serialPortParam->Key + "] - SerialPortParam's Name was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's Name was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取BaudRate参数
    if ((!fileStorage["BaudRate"].isNone()) && (fileStorage["BaudRate"].isInt()))
    {
        EBaudRate baudRate;
        if (SerialPortParam::ConvertToBaudRate(static_cast<int>(fileStorage["BaudRate"]), &baudRate))
        {
            serialPortParam->BaudRate = baudRate;
            log = "[" + serialPortParam->Key + "] - SerialPortParam's BaudRate was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + serialPortParam->Key + "] - SerialPortParam's BaudRate was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's BaudRate was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取DataBits参数
    if ((!fileStorage["DataBits"].isNone()) && (fileStorage["DataBits"].isInt()))
    {
        EDataBits dataBits;
        if (SerialPortParam::ConvertToDataBits(static_cast<int>(fileStorage["DataBits"]), &dataBits))
        {
            serialPortParam->DataBits = dataBits;
            log = "[" + serialPortParam->Key + "] - SerialPortParam's DataBits was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + serialPortParam->Key + "] - SerialPortParam's DataBits was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's DataBits was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取StopBits参数
    if ((!fileStorage["StopBits"].isNone()) && (fileStorage["StopBits"].isInt()))
    {
        EStopBits stopBits;
        if (SerialPortParam::ConvertToStopBits(static_cast<int>(fileStorage["StopBits"]), &stopBits))
        {
            serialPortParam->StopBits = stopBits;
            log = "[" + serialPortParam->Key + "] - SerialPortParam's StopBits was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + serialPortParam->Key + "] - SerialPortParam's StopBits was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's StopBits was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Parity参数
    if ((!fileStorage["Parity"].isNone()) && (fileStorage["Parity"].isInt()))
    {
        EParity parity;
        if (SerialPortParam::ConvertToParity(static_cast<int>(fileStorage["Parity"]), &parity))
        {
            serialPortParam->Parity = parity;
            log = "[" + serialPortParam->Key + "] - SerialPortParam's Parity was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + serialPortParam->Key + "] - SerialPortParam's Parity was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's Parity was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取FlowControl参数
    if ((!fileStorage["FlowControl"].isNone()) && (fileStorage["FlowControl"].isInt()))
    {
        EFlowControl flowControl;
        if (SerialPortParam::ConvertToFlowControl(static_cast<int>(fileStorage["FlowControl"]), &flowControl))
        {
            serialPortParam->FlowControl = flowControl;
            log = "[" + serialPortParam->Key + "] - SerialPortParam's FlowControl was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + serialPortParam->Key + "] - SerialPortParam's FlowControl was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's FlowControl was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ReadFromDriverCpuCore参数
    if ((!fileStorage["ReadFromDriverCpuCore"].isNone()) && (fileStorage["ReadFromDriverCpuCore"].isInt()))
    {
        serialPortParam->ReadFromDriverCpuCore = static_cast<int>(fileStorage["ReadFromDriverCpuCore"]);
        log = "[" + serialPortParam->Key + "] - SerialPortParam's ReadFromDriverCpuCore was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's ReadFromDriverCpuCore was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取WriteToDriverCpuCore参数
    if ((!fileStorage["WriteToDriverCpuCore"].isNone()) && (fileStorage["WriteToDriverCpuCore"].isInt()))
    {
        serialPortParam->WriteToDriverCpuCore = static_cast<int>(fileStorage["WriteToDriverCpuCore"]);
        log = "[" + serialPortParam->Key + "] - SerialPortParam's WriteToDriverCpuCore was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's WriteToDriverCpuCore was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ReadBufferSize参数
    if ((!fileStorage["ReadBufferSize"].isNone()) && (fileStorage["ReadBufferSize"].isInt()))
    {
        serialPortParam->ReadBufferSize = static_cast<int>(fileStorage["ReadBufferSize"]);
        log = "[" + serialPortParam->Key + "] - SerialPortParam's ReadBufferSize was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's ReadBufferSize was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取WriteBufferSize参数
    if ((!fileStorage["WriteBufferSize"].isNone()) && (fileStorage["WriteBufferSize"].isInt()))
    {
        serialPortParam->WriteBufferSize = static_cast<int>(fileStorage["WriteBufferSize"]);
        log = "[" + serialPortParam->Key + "] - SerialPortParam's WriteBufferSize was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's WriteBufferSize was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Separators参数
    cv::FileNode separatorsNode = fileStorage["Separators"];
    if (!separatorsNode.empty())
    {
        unsigned int separatorIndex = 0;
        cv::FileNodeIterator separatorIterator = separatorsNode.begin();
        while (separatorIterator != separatorsNode.end())
        {
            if ((*separatorIterator).isInt())
            {
                auto separator = static_cast<int>(*separatorIterator);
                if ((separator >= 0) && (separator <= 255))
                {
                    serialPortParam->Separators.emplace_back(separator);
                    log = "[" + serialPortParam->Key + "] - "\
                          "SerialPortParam's Separators[" + std::to_string(separatorIndex) + "] "\
                          "was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + serialPortParam->Key + "] - "\
                          "SerialPortParam's Separators[" + std::to_string(separatorIndex) + "]'s value "\
                          "is invalid";
                    logger.Save(ELogType::Error, log);
                }
            }
            else
            {
                log = "[" + serialPortParam->Key + "] - "\
                      "SerialPortParam's Separators[" + std::to_string(separatorIndex) + "] "\
                      "was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 分隔符索引和迭代器累加
            separatorIndex++;
            separatorIterator++;
        }
    }
    else
    {
        log = "[" + serialPortParam->Key + "] - SerialPortParam's Separators was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + serialPortParam->Key + "] - SerialPortParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}