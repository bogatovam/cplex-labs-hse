
#include <include/cql_graph.h>

#include <fstream>
#include <sstream>

void CqlGraph::readGraph(const std::string &graphsPath, const std::string &graphName) {
    std::fstream file;
    file.open(graphsPath + "/" + graphName, std::fstream::in | std::fstream::out | std::fstream::app);

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
            this->n = std::stoi(words[words.size() - 2]);
            this->m = std::stoi(words[words.size() - 1]);

            break;
        }
    }

    this->adjacency_lists.resize(n, std::vector<int>());
    this->confusion_matrix.resize(n, std::vector<bool>(n, false));
    this->confusion_matrix_bit_set.resize(n, std::bitset<1000>(false));

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'e') {
            continue;
        }

        std::istringstream iss(current_line);
        iss >> type >> current_edge.first >> current_edge.second;

        this->adjacency_lists[current_edge.first - 1].push_back(current_edge.second - 1);
        this->adjacency_lists[current_edge.second - 1].push_back(current_edge.first - 1);

        this->confusion_matrix[current_edge.first - 1][current_edge.second - 1] = true;
        this->confusion_matrix[current_edge.second - 1][current_edge.first - 1] = true;

        this->confusion_matrix_bit_set[current_edge.first - 1][current_edge.second - 1] = true;
        this->confusion_matrix_bit_set[current_edge.second - 1][current_edge.first - 1] = true;
    }
    file.close();
}

int CqlGraph::getN() const {
    return n;
}

void CqlGraph::setN(int n) {
    CqlGraph::n = n;
}

int CqlGraph::getM() const {
    return m;
}

void CqlGraph::setM(int m) {
    CqlGraph::m = m;
}

const std::vector<std::vector<bool>> &CqlGraph::getConfusionMatrix() const {
    return confusion_matrix;
}

void CqlGraph::setConfusionMatrix(const std::vector<std::vector<bool>> &confusionMatrix) {
    confusion_matrix = confusionMatrix;
}

const std::vector<std::vector<int>> &CqlGraph::getAdjacencyLists() const {
    return adjacency_lists;
}

void CqlGraph::setAdjacencyLists(const std::vector<std::vector<int>> &adjacencyLists) {
    adjacency_lists = adjacencyLists;
}

CqlGraph::CqlGraph(const std::string &graphsPath, const std::string &graphName) {
    this->readGraph(graphsPath, graphName);
}
