#include "include/cplex_model.h"
#include "include/cql_graph.h"
#include "csv_writer.h"

namespace max_clique_solver {
    CplexModel init_cplex_model(const CqlGraph &graph);

    void solve(const CqlGraph &graph, const CsvWriter &csv_writer);
}