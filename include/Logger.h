#pragma once

#include <fstream>
#include <string>
#include <mutex>
#include <format>

/**
 * @brief Thread-safe simple logger that writes text lines to a file.
 */
class Logger {
public:
    explicit Logger(const std::string& path);
    ~Logger();

    template <typename... Ts>
    void WriteLine(const std::string& fmt, Ts&&... args) {
        std::scoped_lock lock{mtx_};
        if (!ofs_.is_open()) return;
        ofs_ << std::vformat(fmt, std::make_format_args(std::forward<Ts>(args)...)) << '\n';
    }

private:
    std::ofstream ofs_;
    std::mutex   mtx_;
};
