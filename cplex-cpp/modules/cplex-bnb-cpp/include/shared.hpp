#pragma once

#include <utility>
#include <cstdint>
#include <vector>
#include <bitset>
#include <chrono>
#include <thread>
#include <Windows.h>


bool isNumberInteger(double number) {
    double int_part;
    return modf(number, &int_part) == 0.0;
}

bool isNumberCloseToInteger(double number, double eps = 0.00001) {
    double up = std::ceil(number);
    double down = std::floor(number);

    if (number + eps > up || number - eps < down) {
        return true;
    }
    return false;
}

template<class T>
class Solution {
public:

    double size;

    T values;

    uint64_t integer_variables_num = 0;

    Solution &operator=(const Solution &other) {
        this->size = std::move(other.size);
        this->values = std::move(other.values);
        this->other = std::move(other.integer_variables_num);
        return *this;
    }

    Solution(double size, T values) : size(size), values(values) {
        this->integer_variables_num = countIntegers(values);
    }

    uint64_t countIntegers(const T &result) {
        uint64_t count = 0;
        for (const double &element: result) {
            if (isNumberInteger(element) || isNumberCloseToInteger(element)) {
                count++;
            }
        }
        return count;
    }

    std::set<uint64_t> extractResult() const {
        std::set<uint64_t> clique;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (values[i] == 1.0) {
                clique.insert(i);
            }
        }
        return clique;
    }

    void printInfo() const {
        std::cout << "Solution:\t" << std::endl;
        for (const double &element: values) {
            std::cout << element << ",\t" << std::endl;
        }
        std::cout << std::endl;
        std::cout << "OBJ VALUE:\t" << size << std::endl;
        std::cout << "INTEGERS COUNT:\t" << integer_variables_num << std::endl;
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
        if (!is_time_over) {
            TerminateThread(timer.native_handle(), 0);
        }
        timer.join();
    }

private:
    std::thread timer;
};

std::bitset<1024> asBitset(const std::set<uint64_t> &set) {
    std::bitset<1024> result;
    for (const uint64_t &element: set) {
        result.set(element, true);
    }
    return result;
}
