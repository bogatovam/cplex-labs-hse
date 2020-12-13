#pragma once

#include "cql_graph.h"


class LocalSearchExecutionContext {
public:
    LocalSearchExecutionContext(const CqlGraph &graph,
                                std::bitset<1024> current_solution,
                                std::size_t current_solution_size,
                                std::bitset<1024> non_solution_vertices,
                                std::vector<uint64_t> tightness);

    const CqlGraph &graph;

    std::bitset<1024> current_solution;

    std::size_t current_solution_size;

    std::bitset<1024> non_solution_vertices;

    std::vector<uint64_t> tightness;

    LocalSearchExecutionContext(const std::bitset<1024> &current_solution, const CqlGraph &graph, std::uint64_t init = 0);

    std::vector<uint64_t> calculateTightness(std::bitset<1024> possible_candidates, std::uint64_t init = 0) const;
};

class CliqueLocalSearchExecutionContext : public LocalSearchExecutionContext {
public:
    CliqueLocalSearchExecutionContext(const std::bitset<1024> &current_is,
                                      const CqlGraph &graph);


    CliqueLocalSearchExecutionContext(const CliqueLocalSearchExecutionContext &context);

    CliqueLocalSearchExecutionContext &operator=(const CliqueLocalSearchExecutionContext &other);


    void updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                std::map<uint64_t, std::bitset<1024>> &candidates_1_2_swap);

    std::map<uint64_t, std::bitset<1024>>
    build12SwapCandidatesSet(std::bitset<1024> current_solution,
                             const std::vector<uint64_t> &tightness) const;

    std::pair<uint64_t, uint64_t> findFirst12Swap(std::bitset<1024> subgraph) const;

    void perturb(size_t k = 1);

    void updateSet(std::bitset<1024> deleted, uint64_t inserted);

    void localSearch();
};

class WISLocalSearchExecutionContext : public LocalSearchExecutionContext {
public:
    WISLocalSearchExecutionContext(const std::bitset<1024> &current_is,
                                   const CqlGraph &graph,
                                   const std::vector<double> &weights);


    WISLocalSearchExecutionContext(const WISLocalSearchExecutionContext &context);

    WISLocalSearchExecutionContext &operator=(const WISLocalSearchExecutionContext &other);

    std::vector<double> weights_diff;

    const std::vector<double> &weights;

    std::vector<double> WISLocalSearchExecutionContext::calculateWeightsDiff();

    double weight();

    void localSearch();

    void updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                std::map<uint64_t, std::bitset<1024>> &candidates_1_2_swap);

    std::map<uint64_t, std::bitset<1024>>
    build12SwapCandidatesSet(std::bitset<1024> current_solution,
                             const std::vector<uint64_t> &tightness) const;

    std::pair<uint64_t, uint64_t> findFirst12Swap(double w_to_delete, const std::bitset<1024> &candidates) const;

    void perturb(size_t k = 1);

    void updateSet(std::bitset<1024> deleted, uint64_t inserted);
};

class LocalSearchLauncher {
public:
    static std::pair<double, std::bitset<1024>> localSearch(std::bitset<1024> initial_solution, const CqlGraph &graph,
                                                            const std::vector<double> &weights);

    static std::pair<uint64_t, std::bitset<1024>> localSearch(std::bitset<1024> initial_solution, const CqlGraph &graph);
};
