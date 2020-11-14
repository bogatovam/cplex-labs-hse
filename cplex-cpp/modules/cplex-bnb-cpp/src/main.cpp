#include <iostream>
#include <include/cql_graph.h>
#include <include/constants.h>
#include <include/csv_writer.h>
#include <include/utils.h>

int main() {
    std::vector<std::string> test = {"name", "time"};

    CsvWriter csv_log("./", "results-" + utils::get_current_datetime_str() + ".csv", test);

    for (const std::string &graph_name: GRAPHS_NAMES) {
        CqlGraph graph = CqlGraph::readGraph("../../../graphs", graph_name);
        std::cout << graph.n_ << std::endl;
    }
    return 0;
}