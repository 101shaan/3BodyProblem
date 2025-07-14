#include "Logger.h"
#include <format>

Logger::Logger(const std::string& path)
    : ofs_{path, std::ios::out | std::ios::trunc}
{
}

Logger::~Logger() {
    if (ofs_.is_open()) ofs_.close();
}
