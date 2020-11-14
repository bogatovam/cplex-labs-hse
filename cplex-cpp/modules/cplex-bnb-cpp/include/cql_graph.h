#pragma once

#include <iostream>
#include <vector>
#include <bitset>

class CqlGraph {
private:

public:
    const uint64_t n_;
    const uint64_t m_;
    const std::vector<std::vector<bool>> confusion_matrix_;
    const std::vector<std::vector<uint64_t>> adjacency_lists_;
    // just big number
    const std::vector<std::bitset<1000>> confusion_matrix_bit_set_;

    CqlGraph(uint64_t n,
             uint64_t m,
             std::vector<std::vector<bool>> matrix,
             std::vector<std::vector<uint64_t>> lists,
             std::vector<std::bitset<1000>> matrix_b);

    static CqlGraph readGraph(const std::string &graphs_path, const std::string &graph_name);

};