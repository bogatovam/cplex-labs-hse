#include <algorithm>
#include "include/max_clique_solver.h"
#include <chrono>

using namespace std::chrono;

CplexModel max_clique_solver::init_cplex_model(const CqlGraph &graph) {
//    CplexModel cplex_solver(graph.n_);


    return CplexModel();
}

void max_clique_solver::solve(const CqlGraph &graph, const CsvWriter &csv_writer, const Strategy &calc_strategy) {
    CplexModel cplex_solver = init_cplex_model(graph);

    std::size_t max_possible_clique = graph.maxDegree();
    std::map<NodesOrderingStrategy, std::vector<uint64_t>> coloring_by_strategy;

    std::set<uint64_t> best_clique;
    for (auto coloring_strategy: nodes_ordering_strategies) {

        if (best_clique.size() == max_possible_clique) {
            break;
        }

        steady_clock::time_point begin = steady_clock::now();
        std::vector<uint64_t> current_coloring = graph.colorGraph(coloring_strategy);
        steady_clock::time_point end = steady_clock::now();
        std::cout << "Graph coloring time = " << duration_cast<milliseconds>(end - begin).count() << "[ms]"
                  << std::endl;

        coloring_by_strategy[coloring_strategy] = current_coloring;

        begin = steady_clock::now();
        std::set<uint64_t> current_clique = graph.getHeuristicMaxClique(current_coloring);
        end = steady_clock::now();
        std::cout << "Max clique BITSET time = " << duration_cast<milliseconds>(end - begin).count() << "[ms]"
                  << std::endl;

        if (current_clique.size() > best_clique.size()) {
            best_clique = current_clique;
        }
    }
}

std::set<uint64_t> max_clique_solver::getBestMaxClique() {
    //    std::size_t max_clique_upperbound = *std::max_element(degrees_.begin(), degrees_.end());
    return std::set<uint64_t>();
}
