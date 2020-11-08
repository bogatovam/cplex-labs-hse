
#include <include/cql_graph.h>

#include <utility>
#include <fstream>
#include <sstream>
#include <algorithm>

CqlGraph CqlGraph::readGraph(const std::string &graphsPath, const std::string &graphName) {
    int n = -1, m = -1;

    std::fstream file;
    file.open("test.txt", std::fstream::in | std::fstream::out | std::fstream::app);

    std::string type;
    std::string current_line;
    std::pair<int, int> current_edge;

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'p') {
            continue;
        } else {
            std::reverse(current_line.begin(), current_line.end());
            std::istringstream iss(current_line);
            iss >> m >> n;

            break;
        }
    }

    std::vector<std::vector<int>> adjacencyLists(n, std::vector<int>());
    std::vector<std::vector<bool>> confusionMatrix(n, std::vector<bool>(n, false));

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'e') {
            continue;
        }
        std::istringstream iss(current_line);
        iss >> current_edge.first >> current_edge.second;

        adjacencyLists[current_edge.first - 1].push_back(current_edge.second - 1);
        adjacencyLists[current_edge.second - 1].push_back(current_edge.first - 1);

        confusionMatrix[current_edge.first - 1][current_edge.second - 1] = true;
        confusionMatrix[current_edge.second - 1][current_edge.first - 1] = true;
    }
    file.close();

    return CqlGraph(confusionMatrix, adjacencyLists, n, m);
}

CqlGraph::CqlGraph(std::vector<std::vector<bool>> confusionMatrix,
                   std::vector<std::vector<int>> adjacencyLists, int n, int m) : confusion_matrix(std::move(
        confusionMatrix)), adjacency_lists(std::move(adjacencyLists)), n(n), m(m) {}
