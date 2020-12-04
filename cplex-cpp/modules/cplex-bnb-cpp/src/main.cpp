#include <iostream>
#include <include/csv_writer.h>
#include <include/utils.h>
#include <include/cql_graph.h>
#include <include/constants.h>
#include <include/max_clique_solver.h>


int main() {

    std::vector<std::string> test = {"graph", "best possible solution", "heuristic_result", "heuristic_time (sec)",
                                     "heuristic_time (ms)", "cplex time (sec)", "cplex time (ms)", "result",
                                     "result", "max_depth", "branches_num", "average_float_cplex_time",
                                     "float_cplex_time"};

    CsvWriter csv_log("./", "results-" + utils::get_current_datetime_str() + ".csv", test);
    for (const auto &graph_name_and_best_solution: GRAPHS_NAMES) {
        CqlGraph graph = CqlGraph::readGraph("../../../graphs", graph_name_and_best_solution.first);
//      graph::CqlGraph graph = graph::CqlGraph::readGraph("../../../graphs", graph_name);
        std::cout << "\n\n" + graph_name_and_best_solution.first << "\t vertices number = " << graph.n_ << std::endl;
        auto log = max_clique_solver::solve(graph, max_clique_solver::Strategy::BRANCH_AND_BOUND);

        log["graph"] = graph_name_and_best_solution.first;
        log["best possible solution"] = std::to_string(graph_name_and_best_solution.second);
        csv_log.write_row(log);
    }
    return 0;
}