#pragma once

#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace utils {
    static std::string get_current_datetime_str() {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%d-%m_-%Y-%H-%M-%S");

        return oss.str();
    }
}