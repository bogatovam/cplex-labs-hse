#pragma once

#include <utility>
#include <cstdint>
#include <vector>
#include <bitset>
#include <chrono>
#include <thread>
#include <Windows.h>

template<class T>
class Solution {
public:

    double size;

    T values;

    Solution &operator=(const Solution &other) {
        this->size = std::move(other.size);
        this->values = std::move(other.values);
        return *this;
    }
};

typedef Solution<std::vector<double>> FloatSolution;
typedef Solution<std::bitset<1024>> IntegerBitSolution;
typedef Solution<std::vector<uint64_t>> IntegerSolution;
typedef std::pair<uint64_t, uint64_t> BranchingWays;

class Timer {
public:
    std::atomic_bool is_time_over;

    explicit Timer(std::chrono::steady_clock::duration time_to_execute) {
        auto timer_func = [&]() {
            std::this_thread::sleep_for(time_to_execute);
            is_time_over = true;
        };
        timer = std::thread(timer_func);
        is_time_over = false;
    }

    ~Timer() {
        if (!is_time_over)
            TerminateThread(timer.native_handle(), 0);
        timer.join();
    }

private:
    std::thread timer;
};