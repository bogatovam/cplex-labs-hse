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

typedef std::pair<double, Column> WeightWithColumn;

typedef std::map<NodesOrderingStrategy, std::vector<uint64_t>> ColoringByStrategy;

typedef std::set<Column> IndependentSets;

static const std::vector<NodesOrderingStrategy> nodes_ordering_strategies = {
        NodesOrderingStrategy::SATURATION_SMALLEST_FIRST,
        NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST,
//        NodesOrderingStrategy::LARGEST_DEGREE_FIRST,
//        NodesOrderingStrategy::SMALLEST_DEGREE_FIRST,
        NodesOrderingStrategy::RANDOM,
        NodesOrderingStrategy::INDEX
};

class Graph {
public:
    const std::size_t n_;
    const std::size_t m_;

    const std::vector<std::set<uint64_t>> adjacency_lists_;
    const std::vector<Bitset> confusion_matrix_bit_set_;

    Graph(
            std::size_t n,
            std::size_t m,
            std::vector<std::set<uint64_t>> lists,
            std::vector<Bitset> matrix_b);

    static Graph readGraph(const std::string &graphs_path, const std::string &graph_name);

    std::pair<std::vector<uint64_t>, uint64_t> colorGraph(const NodesOrderingStrategy &strategy) const;

    std::vector<uint64_t> orderVertices(const NodesOrderingStrategy &strategy) const;

    std::vector<uint64_t> orderVerticesLatestDegreeFirst(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> orderVerticesSmallestDegreeFirst(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> orderVerticesSmallestDegreeSupportFirst(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> orderVerticesRandom(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> orderVerticesSaturationSmallestFirst(std::vector<uint64_t> vertices) const;

    std::vector<uint64_t> verticesSupport() const;

    bool isColoringCorrect(std::vector<uint64_t> coloring) const;

    std::set<uint64_t> getHeuristicMaxClique(const std::vector<uint64_t> &coloring, NodesOrderingStrategy cs) const;

    bool isClique(const std::set<uint64_t> &clique) const;

    std::size_t degree(uint64_t vertex) const {
        return adjacency_lists_[vertex].size();
    }

    std::size_t maxDegree() const {
        std::size_t max_degree = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            if (degree(i) > max_degree) {
                max_degree = degree(i);
            }
        }
        return max_degree;
    }

    Graph buildSubgraph(Bitset set) const;

    Graph buildComplementGraph() const;

    Bitset getHeuristicMaxCliqueRecursive(const std::vector<uint64_t> &coloring, NodesOrderingStrategy cs) const;

    bool isClique(const Bitset &clique) const;

    uint64_t findCliqueBestCandidate(const std::vector<uint64_t> &vector, Graph graph) const;

    bool isVerticesIndependent(std::set<uint64_t> &independent_set) const;

    bool isVerticesIndependent(Column &independent_set) const;

    std::set<std::pair<double, std::set<uint64_t >>> findWeightedIndependentSet(
            const std::vector<double> &weights) const;

    std::vector<uint64_t> orderVerticesSaturationSmallestFirstWeighted(
            std::vector<uint64_t> vertices,
            const std::vector<double> &wights) const;

    std::map<uint64_t, std::set<uint64_t>> colorWeightedGraph(const std::vector<double> &wights) const;

    IndependentSets getIndependentSetByColoring(const NodesOrderingStrategy &strategy) const;

    std::pair<bool, Column> supplementSetsToMaximumForInclusion(const Column &independent_set) const;
};
