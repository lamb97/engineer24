#include "serial_port_param.h"

int main(int, char* [])
{
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    SerialPortParam serialPortParam;
    SerialPortParam::LoadFromYamlFile("config/basement/serial_port_param.yaml",
                                      &serialPortParam);

    return 0;
}