#include "include/cplex_model.h"
#include "include/cql_graph.h"
#include "csv_writer.h"

namespace max_clique_solver {
    enum class Strategy {
        BRANCH_AND_CUT = 0,
        BRANCH_AND_BOUND = 1,
    };

    CplexModel
    init_cplex_model(const CqlGraph &graph, const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &map);

    std::map<std::string, std::string> solve(const CqlGraph &graph, const Strategy &calc_strategy);

    void branch_and_cut();

    std::bitset<1024> getBestMaxClique(const CqlGraph &graph,
                                       std::map<NodesOrderingStrategy, std::vector<uint64_t>>& coloring_by_strategy);

    std::set<std::set<uint64_t>> buildAdjacencyConstrains(const CqlGraph &graph);

    std::set<std::set<uint64_t>> buildColoringConstrains(const CqlGraph &graph,
                                                         const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring);

    void improveIndependentSet(const CqlGraph &graph, std::set<uint64_t> &independent_set);


}