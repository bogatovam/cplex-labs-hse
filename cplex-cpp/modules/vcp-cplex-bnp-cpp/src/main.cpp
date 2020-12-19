#include <iostream>
#include <include/csv_writer.h>
#include <include/utils.h>
#include <include/cql_graph.h>
#include <include/constants.h>
#include <include/vertex_coloring_solver.h>


int main() {

    std::vector<std::string> test = {"graph", "best possible solution", "result", "heuristic_result", "timeout",
                                     "time (sec)", "max_depth", "branches_num", "average_float_cplex_time",
                                     "discarded_branches_num", "average_cutting_time",
                                     "average_cutting_iteration_time"};

    CsvWriter csv_log("./", "results-" + utils::get_current_datetime_str() + ".csv", test);
    csv_log.writeTitle("");
    for (const auto &graph_name_and_best_solution: GRAPHS_NAMES) {
        Graph graph = Graph::readGraph("../../../graphs", graph_name_and_best_solution.first);
        std::cout << "\n\n" + graph_name_and_best_solution.first << "\t vertices number = " << graph.n_ << std::endl;
        auto log = vertex_coloring_solver::solve(graph, vertex_coloring_solver::Strategy::BRANCH_AND_CUT);

        log["graph"] = graph_name_and_best_solution.first;
        log["best possible solution"] = std::to_string(graph_name_and_best_solution.second);
        csv_log.writeRow(log);
    }
    return 0;
}