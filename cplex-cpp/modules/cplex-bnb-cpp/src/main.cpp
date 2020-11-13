#include <iostream>
#include <include/cql_graph.h>
#include <include/configuration.h>
#include <include/csv_writer.h>

using namespace std;

int main() {
    for (const std::string &graph_name: GRAPHS_NAMES) {
        CqlGraph graph = CqlGraph("../../../graphs", graph_name);
        std::cout << graph.getN() << std::endl;
    }
    std::vector<std::string> test = {"name", "time"};

    CsvWriter csv_log("./", "results.csv", test);
    std::map<std::string, std::string> m = {{"name", "test"},
                                            {"time", "test2"}};
    csv_log.write_row(m);
    csv_log.write_row(m);
    return 0;
}