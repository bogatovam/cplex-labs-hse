#include <algorithm>
#include "include/max_clique_solver.h"
#include <chrono>

//#define CHECK_SOLUTION

using namespace std::chrono;

std::set<std::set<uint64_t>> max_clique_solver::buildAdjacencyConstrains(const CqlGraph &graph) {
    std::set<std::set<uint64_t>> result;

//  add constrains for every i,j: x_i + x_j <= 1 if E(i,j) = 0
    for (std::size_t v = 0; v < graph.n_; ++v) {
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (!graph.confusion_matrix_bit_set_[v][u]) {
                std::set<uint64_t> constrain;
                constrain.insert(u);
                constrain.insert(v);
                result.insert(constrain);
            }
        }
    }

    //  add constrains for every v_i .. v_j: x_i + ... +  x_j <= 1 if E(v_i .. v_j) = 0
    std::vector<uint64_t> ordered = graph.orderVertices(NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST);

    auto vertices_order = [&](uint64_t i, uint64_t j) {
        return (graph.degree(i) < graph.degree(j)) || (graph.degree(i) == graph.degree(j) && i < j);
    };

    std::vector<std::vector<uint64_t>> independent_vertices_per_node(graph.n_);

    for (uint32_t v = 0; v < graph.n_; ++v) {
        for (uint32_t u = v + 1; u < graph.n_; ++u) {
            if (u != v && !graph.confusion_matrix_bit_set_[u][v]) {
                independent_vertices_per_node[u].push_back(v);
                independent_vertices_per_node[v].push_back(u);
            }
        }
    }

    for (uint32_t v = 0; v < graph.n_; ++v) {
        std::sort(independent_vertices_per_node[v].begin(), independent_vertices_per_node[v].end(), vertices_order);
    }

    for (uint64_t v: ordered) {
        std::set<uint64_t> constrain;

        constrain.emplace(v);
        std::vector<uint64_t> candidates = independent_vertices_per_node[v];

        while (!candidates.empty()) {
            uint64_t inserted_vertex = *candidates.begin();

            constrain.emplace(*candidates.begin());
            std::vector<uint64_t> new_candidates;
            std::set_intersection(candidates.begin(), candidates.end(),
                                  independent_vertices_per_node[inserted_vertex].begin(),
                                  independent_vertices_per_node[inserted_vertex].end(),
                                  inserter(new_candidates, new_candidates.begin()));
            candidates = new_candidates;
        }
        if (constrain.size() > 2) {
            result.emplace(constrain);
        }
    }
    return result;
}


std::set<std::set<uint64_t>> max_clique_solver::buildColoringConstrains(const CqlGraph &graph,
                                                                        const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring) {
    std::set<std::set<uint64_t>> result;
    std::map<uint64_t, std::set<uint64_t >> vertices_by_color;
    for (const auto &type_to_coloring : coloring) {
        vertices_by_color.clear();

        for (std::size_t v = 0; v < graph.n_; ++v) {
            vertices_by_color[type_to_coloring.second[v]].insert(v);
        }

        for (auto independent_set: vertices_by_color) {
            if (independent_set.second.size() <= 2)
                continue;
            std::set<uint64_t> constrain(independent_set.second.begin(), independent_set.second.end());

            // it really helps in some graphs
            std::set<std::set<uint64_t>> improvements = improveIndependentSet(graph, constrain);
            if (improvements.empty()) {
                result.emplace(std::set<uint64_t>{
                        constrain.begin(),
                        constrain.end()
                });
            }
            result.insert(improvements.begin(), improvements.end());
        }
    }
    return result;
}


std::set<std::set<uint64_t>> max_clique_solver::improveIndependentSet(const CqlGraph &graph,
                                                                      const std::set<uint64_t> &independent_set) {
    std::set<std::set<uint64_t>> result;
    std::set<uint64_t> can_be_added;
    std::bitset<1024> independent_set_bit;

    for (auto v: independent_set) {
        independent_set_bit.set(v, true);
    }

    for (auto v: independent_set) {
        independent_set_bit |= graph.confusion_matrix_bit_set_[v];
    }

    bool hasImprovements = !independent_set_bit.all();
    if (hasImprovements) {
        std::bitset<1024> dop = ~independent_set_bit;
        CqlGraph subgraph = graph.buildSubgraph(dop);
        auto coloring = subgraph.colorGraph(NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST);
        std::map<uint64_t, std::set<uint64_t >> vertices_by_color;
        for (std::size_t v = 0; v < graph.n_; ++v) {
            vertices_by_color[coloring.first[v]].insert(v);
        }
        for (const auto &nested_independent_set: vertices_by_color) {
            std::set<uint64_t> constrain;
            for (auto v: nested_independent_set.second) {
                if (dop[v]) {
                    constrain.insert(v);
                }
            }
            constrain.insert(independent_set.begin(), independent_set.end());

#ifdef CHECK_SOLUTION
            std::cout<<"CHECK"<<std::endl;
            if (!graph.isVerticesIndependent(constrain)) {
                std::cout << "set is not independent" << std::endl;
                throw std::runtime_error("set is not independent");
            }
#endif
            result.insert(constrain);
        }
    }

    return result;
}

CplexModel max_clique_solver::init_cplex_model(const CqlGraph &graph,
                                               const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring) {
    CplexModel cplex_solver(graph.n_);

    std::set<std::set<uint64_t>> constrains;
    std::set<std::set<uint64_t>> adjacencyConstrains = buildAdjacencyConstrains(graph);
    std::set<std::set<uint64_t>> coloringConstrains = buildColoringConstrains(graph, coloring);

    constrains.insert(adjacencyConstrains.begin(), adjacencyConstrains.end());
    constrains.insert(coloringConstrains.begin(), coloringConstrains.end());

    cplex_solver.addConstraints(constrains, 0, 1);
    return cplex_solver;
}

std::map<std::string, std::string> max_clique_solver::solve(const CqlGraph &graph, const Strategy &calc_strategy) {
    std::map<std::string, std::string> log;
    std::map<NodesOrderingStrategy, std::vector<uint64_t>> coloring_by_strategy;

    steady_clock::time_point begin = steady_clock::now();
    std::bitset<1024> best_clique = getBestMaxClique(graph, coloring_by_strategy);

    CplexModel cplex_solver = init_cplex_model(graph, coloring_by_strategy);
    auto execution_time = steady_clock::now() - begin;
    log["heuristic_time (sec)"] = std::to_string(duration_cast<seconds>(execution_time).count());
    log["heuristic_time (ms)"] = std::to_string(duration_cast<milliseconds>(execution_time).count());
    log["heuristic_result"] = std::to_string(best_clique.count());

    begin = steady_clock::now();
    cplex_solver.solveInteger(graph);
    execution_time = steady_clock::now() - begin;
    log["cplex time (sec)"] = std::to_string(duration_cast<seconds>(execution_time).count());
    log["cplex time (ms)"] = std::to_string(duration_cast<milliseconds>(execution_time).count());

    return log;
}

std::bitset<1024> max_clique_solver::getBestMaxClique(const CqlGraph &graph,
                                                      std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring_by_strategy) {
    std::bitset<1024> best_clique;
    std::size_t max_possible_clique = graph.maxDegree() + 1;

    for (auto coloring_strategy: nodes_ordering_strategies) {

        if (best_clique.size() == max_possible_clique) {
            break;
        }

        auto coloring_to_max_color = graph.colorGraph(coloring_strategy);
        coloring_by_strategy[coloring_strategy] = coloring_to_max_color.first;
        std::bitset<1024> current_clique = graph.getHeuristicMaxCliqueRecursive(coloring_to_max_color.first,
                                                                                coloring_strategy);
        if (current_clique.count() > best_clique.count()) {
            best_clique = current_clique;
        }
    }
    best_clique = graph.localSearch(best_clique);
    return best_clique;
}
