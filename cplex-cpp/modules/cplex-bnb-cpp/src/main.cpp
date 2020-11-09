#include <iostream>
#include <include/cql_graph.h>
#include <include/configuration.h>

using namespace std;

int main() {
    for (const std::string &graph_name: GRAPHS_NAMES) {
        CqlGraph graph = CqlGraph("../../../graphs", graph_name);
        std::cout << graph.getN()<< std::endl;
    }
    return 0;
}