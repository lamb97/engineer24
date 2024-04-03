find_package(Threads REQUIRED)

# 生成logger库
add_library(logger SHARED
            src/easy_logger.cpp
            src/easy_logger_param.cpp)
target_link_libraries(logger
                      Threads::Threads)

# 生成test_logger测试程序
add_executable(test_easy_logger
               test/test_easy_logger.cpp)
target_link_libraries(test_easy_logger
                      logger)

# 生成test_spdlog测试程序
add_executable(test_spdlog
               test/test_spdlog.cpp)
target_link_libraries(test_spdlog
                      Threads::Threads)