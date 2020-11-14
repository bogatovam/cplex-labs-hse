#pragma once

#include <iostream>
#include <vector>
#include <bitset>
#include <set>

class CqlGraph {
private:
    enum class NodesOrderingStrategy {
        INDEX,
        LARGEST_DEGREE_FIRST,
        SMALLEST_DEGREE_FIRST,
        SMALLEST_DEGREE_SUPPORT_FIRST,
        RANDOM,
        SATURATION_LARGEST_FIRST
    };

    const std::vector<NodesOrderingStrategy> nodes_ordering_strategies = {NodesOrderingStrategy::INDEX,
                                                                          NodesOrderingStrategy::LARGEST_DEGREE_FIRST,
                                                                          NodesOrderingStrategy::SMALLEST_DEGREE_FIRST,
                                                                          NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST,
                                                                          NodesOrderingStrategy::RANDOM,
                                                                          NodesOrderingStrategy::SATURATION_LARGEST_FIRST,
    };
public:
    const std::size_t n_;
    const std::size_t m_;
    const std::vector<std::size_t> degrees_;

    const std::vector<std::vector<bool>> confusion_matrix_;
    const std::vector<std::vector<uint64_t>> adjacency_lists_;
    // just big number
    const std::vector<std::bitset<1000>> confusion_matrix_bit_set_;

    CqlGraph(std::size_t n,
             std::size_t m,
             std::vector<std::size_t> degrees,
             std::vector<std::vector<bool>> matrix,
             std::vector<std::vector<uint64_t>> lists,
             std::vector<std::bitset<1000>> matrix_b);

    static CqlGraph readGraph(const std::string &graphs_path, const std::string &graph_name);

    std::set<uint64_t> getHeuristicMaxClique(const std::set<std::set<uint64_t>> &colorings);


    std::set<uint64_t> colorGraph(const NodesOrderingStrategy &strategy) const;

    std::vector<uint64_t> orderVertices(const NodesOrderingStrategy &strategy) const;

    std::vector<uint64_t> orderVerticesLatestDegreeFirst(const std::vector<uint64_t> &vertices) const;

    std::vector<uint64_t> orderVerticesSmallestDegreeFirst(const std::vector<uint64_t> &vertices) const;

    std::vector<uint64_t> orderVerticesSmallestDegreeSupportFirst(const std::vector<uint64_t>& vertices) const;

    std::vector<uint64_t> orderVerticesRandom(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> orderVerticesSaturationLargestFirst(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> verticesSupport() const;
};