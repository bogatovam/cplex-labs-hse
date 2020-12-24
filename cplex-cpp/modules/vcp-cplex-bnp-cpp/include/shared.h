#pragma once

#include <utility>
#include <cstdint>
#include <vector>
#include <bitset>
#include <set>
#include <chrono>
#include <thread>
#include <Windows.h>
#include <atomic>
#include <iostream>

#define CHECK_SOLUTION

typedef std::bitset<1024> Bitset;

Bitset asBitset(const std::set<uint64_t> &set);

std::set<uint64_t> asSet(const Bitset &set, std::size_t n);

std::set<uint64_t> asSet(const std::vector<uint64_t> &vector);

bool isNumberInteger(double number);

bool isNumberCloseToInteger(double number, double eps = 0.00001);

class FloatSolution {
public:

    double size;

    std::vector<double> values;

    uint64_t integer_variables_num = 0;

    FloatSolution &operator=(const FloatSolution &other) = default;

    FloatSolution(double size, const std::vector<double> &values) : size(size), values(values) {
        this->integer_variables_num = countIntegers(values);
    }

    static uint64_t countIntegers(const std::vector<double> &result) {
        uint64_t count = 0;
        for (const double &element: result) {
            if (isNumberInteger(element) || isNumberCloseToInteger(element)) {
                count++;
            }
        }
        return count;
    }

    std::set<uint64_t> extractResult() const {
        std::set<uint64_t> result;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (values[i] == 1.0) {
                result.insert(i);
            }
        }
        return result;
    }
};

class IntegerSolution {
public:

    double upper_bound;

    Bitset values;

    IntegerSolution &operator=(const IntegerSolution &other) = default;

    IntegerSolution(double upper_bound, const Bitset &values) : upper_bound(upper_bound), values(values) {}

    std::set<uint64_t> extractResult() const {
        std::set<uint64_t> result;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (values[i]) {
                result.insert(i);
            }
        }
        return result;
    }

    void print() {
        std::cout << "Slave solution\t(" << upper_bound << "):\t";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (values[i]) {
                std::cout << i << ",\t";
            }
        }
        std::cout << std::endl;
    }
};

class MainFloatSolution {
public:
    FloatSolution primal;

    FloatSolution dual;

    MainFloatSolution &operator=(const MainFloatSolution &other) = default;

    void print();
};

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