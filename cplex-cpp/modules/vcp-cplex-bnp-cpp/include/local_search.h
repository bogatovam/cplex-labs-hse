#pragma once

#include "cql_graph.h"


class LocalSearchExecutionContext {
public:
    LocalSearchExecutionContext(const Graph &graph,
                                Bitset current_solution,
                                std::size_t current_solution_size,
                                Bitset non_solution_vertices,
                                std::vector<uint64_t> tightness);

    const Graph &graph;

    Bitset current_solution;

    std::size_t current_solution_size;

    Bitset non_solution_vertices;

    std::vector<uint64_t> tightness;

    LocalSearchExecutionContext(const Bitset &current_solution, const Graph &graph, std::uint64_t init = 0);

    std::vector<uint64_t> calculateTightness(Bitset possible_candidates, std::uint64_t init = 0) const;
};

class CliqueLocalSearchExecutionContext : public LocalSearchExecutionContext {
public:
    CliqueLocalSearchExecutionContext(const Bitset &current_is,
                                      const Graph &graph);


    CliqueLocalSearchExecutionContext(const CliqueLocalSearchExecutionContext &context);

    CliqueLocalSearchExecutionContext &operator=(const CliqueLocalSearchExecutionContext &other);


    void updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                std::map<uint64_t, Bitset> &candidates_1_2_swap);

    std::map<uint64_t, Bitset>
    build12SwapCandidatesSet(Bitset current_solution,
                             const std::vector<uint64_t> &tightness) const;

    std::pair<uint64_t, uint64_t> findFirst12Swap(Bitset subgraph) const;

    void perturb(size_t k = 1);

    void updateSet(Bitset deleted, uint64_t inserted);

    void localSearch();
};


class ISLocalSearchExecutionContext : public LocalSearchExecutionContext {
public:
    ISLocalSearchExecutionContext(const Bitset &current_is,
                                  const Graph &graph);


    ISLocalSearchExecutionContext(const ISLocalSearchExecutionContext &context);

    ISLocalSearchExecutionContext &operator=(const ISLocalSearchExecutionContext &other);


    void updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                std::map<uint64_t, Bitset> &candidates_1_2_swap);

    std::map<uint64_t, Bitset> build12SwapCandidatesSet(
            Bitset current_solution,
            const std::vector<uint64_t> &tightness) const;

    std::pair<uint64_t, uint64_t> findFirst12Swap(Bitset candidates) const;

    void perturb(size_t k = 1);

    void updateSet(Bitset deleted, uint64_t inserted);

    void localSearch();
};

class WISLocalSearchExecutionContext : public LocalSearchExecutionContext {
public:
    WISLocalSearchExecutionContext(const Bitset &current_is,
                                   const Graph &graph,
                                   const std::vector<double> &weights);


    WISLocalSearchExecutionContext(const WISLocalSearchExecutionContext &context);

    WISLocalSearchExecutionContext &operator=(const WISLocalSearchExecutionContext &other);

//  difference between the weight of v and the total weight of Ns(v)
    std::vector<double> weights_diff;

    const std::vector<double> &weights;

    std::vector<double> calculateWeightsDiff();

    double weight();

    void localSearch();

    void updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                std::map<uint64_t, Bitset> &candidates_1_2_swap);

    std::map<uint64_t, Bitset>
    build12SwapCandidatesSet(Bitset current_solution,
                             const std::vector<uint64_t> &tightness) const;

    std::pair<uint64_t, uint64_t> findFirst12Swap(double w_to_delete, const Bitset &candidates) const;

    void perturb(size_t k = 1);

    void updateSet(Bitset deleted, uint64_t inserted);

    uint64_t getNextW1SwapCandidate() const;
};

class LocalSearchLauncher {
public:
    static WeightWithColumn localSearch(Bitset initial_solution, const Graph &graph,
                                        const std::vector<double> &weights);

    static std::pair<uint64_t, Bitset> localSearch(Bitset initial_solution, const Graph &graph);

    static std::pair<uint64_t, Bitset> independentSetLocalSearch(Bitset initial_solution,
                                                                 const Graph &graph);
};
