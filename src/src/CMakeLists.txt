# 生成uart库
add_library(src SHARED
        src/recognize.cpp)
target_link_libraries(src
        ${OpenCV_LIBS}
        )

# 生成test_serial_port测试程序
add_executable(test1
        test/test1.cpp)
target_link_libraries(test1
        src)