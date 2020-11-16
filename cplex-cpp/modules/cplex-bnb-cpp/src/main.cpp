#include <iostream>
#include <include/csv_writer.h>
#include <include/utils.h>
#include <include/cql_graph.h>
#include <include/constants.h>
#include <include/max_clique_solver.h>

int main() {
    std::vector<std::string> test = {"name", "time"};

    CsvWriter csv_log("./", "results-" + utils::get_current_datetime_str() + ".csv", test);

    for (const std::string &graph_name: TEST_GRAPHS) {
        CqlGraph graph = CqlGraph::readGraph("../../../tests", graph_name);
//      graph::CqlGraph graph = graph::CqlGraph::readGraph("../../../graphs", graph_name);
        std::cout << graph.n_ << std::endl;
        max_clique_solver::solve(graph, csv_log, max_clique_solver::Strategy::BRANCH_AND_BOUND);
    }
    return 0;
}