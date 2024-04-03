#include <iostream>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

int main(int, char* [])
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/multisink.txt", true);
    spdlog::sinks_init_list sink_list = {file_sink, console_sink};
    //spdlog::logger logger("multi_sink", sink_list.begin(), sink_list.end());
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));

    spdlog::info("info message");
    spdlog::warn("warm message");
    spdlog::error("error message");

    auto console_sink1 = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    spdlog::logger logger1("test", console_sink);
    logger1.info("logger1 write info");

    return 0;
}