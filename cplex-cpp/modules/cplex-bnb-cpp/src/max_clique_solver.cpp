#include <algorithm>
#include "include/max_clique_solver.h"
#include <chrono>

using namespace std::chrono;

CplexModel max_clique_solver::init_cplex_model(const CqlGraph &graph) {
//    CplexModel cplex_solver(graph.n_);


    return CplexModel();
}

std::map<std::string, std::string> max_clique_solver::solve(const CqlGraph &graph, const Strategy &calc_strategy) {
    CplexModel cplex_solver = init_cplex_model(graph);
    std::map<std::string, std::string> log;
    std::size_t max_possible_clique = graph.maxDegree() + 1;
    std::map<NodesOrderingStrategy, std::vector<uint64_t>> coloring_by_strategy;

    steady_clock::time_point begin = steady_clock::now();
    std::bitset<1024> best_clique;
    for (auto coloring_strategy: nodes_ordering_strategies) {

        if (best_clique.size() == max_possible_clique) {
            break;
        }

        auto coloring_to_max_color = graph.colorGraph(coloring_strategy);
        coloring_by_strategy[coloring_strategy] = coloring_to_max_color.first;
        std::bitset<1024> current_clique = graph.getHeuristicMaxCliqueRecursive(coloring_to_max_color.first,
                                                                                coloring_strategy);
        std::cout << "Result = " << current_clique.count() << std::endl;

        if (current_clique.count() > best_clique.count()) {
            best_clique = current_clique;
        }
    }
    steady_clock::time_point end = steady_clock::now();

    log["heuristic_time (sec)"] = std::to_string(duration_cast<seconds>(end - begin).count());
    log["heuristic_time (ms)"] = std::to_string(duration_cast<milliseconds>(end - begin).count());
    log["heuristic_result"] = std::to_string(best_clique.count());

    return log;
}

std::set<uint64_t> max_clique_solver::getBestMaxClique() {
    //    std::size_t max_clique_upperbound = *std::max_element(degrees_.begin(), degrees_.end());
    return std::set<uint64_t>();
}
