#pragma once

#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace utils {
    static std::string get_current_datetime_str() {
        std::ostringstream oss;
        oss << std::time(nullptr);
        return oss.str();
    }
}