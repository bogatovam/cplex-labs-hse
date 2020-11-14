
#include <include/cql_graph.h>

#include <fstream>
#include <sstream>

CqlGraph::CqlGraph(const uint64_t n,
                   const uint64_t m,
                   std::vector<std::vector<bool>> matrix,
                   std::vector<std::vector<uint64_t>> lists,
                   std::vector<std::bitset<1000>> matrix_b) : n_(n),
                                                              m_(m),
                                                              confusion_matrix_(std::move(matrix)),
                                                              adjacency_lists_(std::move(lists)),
                                                              confusion_matrix_bit_set_(std::move(matrix_b)) {}

CqlGraph CqlGraph::readGraph(const std::string &graphs_path, const std::string &graph_name) {
    uint64_t n = 0;
    uint64_t m = 0;
    std::vector<std::vector<bool>> confusion_matrix;
    std::vector<std::vector<uint64_t>> adjacency_lists;
    // just big number
    std::vector<std::bitset<1000>> confusion_matrix_bit_set;

    std::fstream file;
    file.open(graphs_path + "/" + graph_name, std::fstream::in | std::fstream::out | std::fstream::app);

    std::string type;
    std::string current_line;
    std::pair<int, int> current_edge;

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'p') {
            continue;
        } else {
            std::istringstream iss(current_line);
            std::vector<std::string> words;

            std::string tmp;
            while ((iss >> tmp)) {
                words.push_back(tmp);
            }
            n = std::stoul(words[words.size() - 2]);
            m = std::stoul(words[words.size() - 1]);

            break;
        }
    }

    adjacency_lists.resize(n, std::vector<uint64_t>());
    confusion_matrix.resize(n, std::vector<bool>(n, false));
    confusion_matrix_bit_set.resize(n, std::bitset<1000>(false));

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'e') {
            continue;
        }

        std::istringstream iss(current_line);
        iss >> type >> current_edge.first >> current_edge.second;

        adjacency_lists[current_edge.first - 1].push_back(current_edge.second - 1);
        adjacency_lists[current_edge.second - 1].push_back(current_edge.first - 1);

        confusion_matrix[current_edge.first - 1][current_edge.second - 1] = true;
        confusion_matrix[current_edge.second - 1][current_edge.first - 1] = true;

        confusion_matrix_bit_set[current_edge.first - 1][current_edge.second - 1] = true;
        confusion_matrix_bit_set[current_edge.second - 1][current_edge.first - 1] = true;
    }
    file.close();
    return CqlGraph(n, m, confusion_matrix, adjacency_lists, confusion_matrix_bit_set);
}
