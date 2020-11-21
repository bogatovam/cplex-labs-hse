#include "include/cplex_model.h"
#include "include/cql_graph.h"
#include "csv_writer.h"

namespace max_clique_solver {
    enum class Strategy {
        BRANCH_AND_CUT = 0,
        BRANCH_AND_BOUND = 1,
    };

    CplexModel init_cplex_model(const CqlGraph &graph);

    std::map<std::string, std::string> solve(const CqlGraph &graph, const Strategy &calc_strategy);

    void branch_and_cut();

    std::set<uint64_t> getBestMaxClique();
}