#pragma once

#include <iostream>
#include <vector>
#include <bitset>
#include <set>
#include <map>
#include "shared.h"

enum class NodesOrderingStrategy {
    INDEX,
    LARGEST_DEGREE_FIRST,
    SMALLEST_DEGREE_FIRST,
    SMALLEST_DEGREE_SUPPORT_FIRST,
    RANDOM,
    SATURATION_SMALLEST_FIRST,
    SMALLEST_DEGREE_SUPPORT_FIRST_WEIGHTED
};
typedef Bitset Column;

typedef std::vector<Column> IndependentSets;

static const std::vector<NodesOrderingStrategy> nodes_ordering_strategies = {
        NodesOrderingStrategy::SATURATION_SMALLEST_FIRST,
        NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST,
        NodesOrderingStrategy::RANDOM,
        NodesOrderingStrategy::INDEX
};

class WeightWithColumn {
public:
    double weight;

    Column column;

    WeightWithColumn(double weight, const Column &column);

    WeightWithColumn &operator=(const WeightWithColumn &other) = default;

    bool operator<(const WeightWithColumn &rhs) const;
};

class Graph {
public:
    const std::size_t n_;
    const std::size_t m_;

    const std::vector<std::set<uint64_t>> adjacency_lists_;
    const std::vector<Bitset> confusion_matrix_bit_set_;

    std::vector<uint64_t> vertices;

    Graph(std::size_t n,
          std::size_t m,
          std::vector<std::set<uint64_t>> lists,
          std::vector<Bitset> matrix_b);

    static Graph readGraph(const std::string &graphs_path, const std::string &graph_name);

    IndependentSets colorWeightedGraph(const std::vector<double> &weights) const;

    IndependentSets getIndependentSetByColoring(const NodesOrderingStrategy &strategy) const;

    std::set<WeightWithColumn> getWeightedIndependentSet(const std::vector<double> &weights) const;

    Column supplementSetsToMaximumForInclusion(const Column &independent_set) const;

    std::vector<uint64_t> verticesSupport() const;

    std::vector<uint64_t> orderVerticesRandom() const;

    std::vector<uint64_t> orderVerticesLatestDegreeFirst() const;

    std::vector<uint64_t> orderVerticesSmallestDegreeFirst() const;

    std::vector<uint64_t> orderVerticesSaturationSmallestFirst() const;

    std::vector<uint64_t> orderVerticesSmallestDegreeSupportFirst() const;

    std::vector<uint64_t> orderVertices(const NodesOrderingStrategy &strategy) const;

    std::vector<uint64_t> orderVerticesSaturationSmallestFirstWeighted(const std::vector<double> &weights) const;

    Graph buildComplementGraph() const;

    bool isClique(const Bitset &clique) const;

    bool isVerticesIndependent(const Column &independent_set) const;

    bool isColoringCorrect(std::vector<uint64_t> coloring) const;

    bool isVerticesIndependent(std::set<uint64_t> &independent_set) const;

    inline std::size_t degree(uint64_t vertex) const {
        return adjacency_lists_[vertex].size();
    }
};