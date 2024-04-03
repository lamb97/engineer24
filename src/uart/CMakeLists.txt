# 生成uart库
add_library(uart SHARED
            src/serial_port.cpp
            src/serial_port_param.cpp)
target_link_libraries(uart
                      ${OpenCV_LIBS}
                      logger)

# 生成test_serial_port测试程序
add_executable(test_serial_port
               test/test_serial_port.cpp)
target_link_libraries(test_serial_port
                      uart)

# 生成test_serial_port_param测试程序
add_executable(test_serial_port_param
               test/test_serial_port_param.cpp)
target_link_libraries(test_serial_port_param
                      uart)