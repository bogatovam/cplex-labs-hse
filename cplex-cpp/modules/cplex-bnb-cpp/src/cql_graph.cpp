
#include <include/cql_graph.h>

#include <fstream>
#include <sstream>
#include <utility>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <random>
#include <vector>
#include <map>
#include <iterator>
#include <limits>

CqlGraph::CqlGraph(const std::size_t n,
                   const std::size_t m,
                   std::vector<std::size_t> degrees,
                   std::vector<std::vector<bool>> matrix,
                   std::vector<std::vector<uint64_t>> lists,
                   std::vector<std::bitset<1000>> matrix_b) : n_(n),
                                                              m_(m),
                                                              degrees_(std::move(degrees)),
                                                              confusion_matrix_(std::move(matrix)),
                                                              adjacency_lists_(std::move(lists)),
                                                              confusion_matrix_bit_set_(std::move(matrix_b)) {}

CqlGraph CqlGraph::readGraph(const std::string &graphs_path, const std::string &graph_name) {
    std::size_t n = 0;
    std::size_t m = 0;
    std::vector<std::size_t> degrees;
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

    degrees.assign(n, 0);
    adjacency_lists.resize(n, std::vector<uint64_t>());
    confusion_matrix.resize(n, std::vector<bool>(n, false));
    confusion_matrix_bit_set.resize(n, std::bitset<1000>(false));

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'e') {
            continue;
        }

        std::istringstream iss(current_line);
        iss >> type >> current_edge.first >> current_edge.second;

        degrees[current_edge.first - 1]++;
        degrees[current_edge.second - 1]++;

        adjacency_lists[current_edge.first - 1].push_back(current_edge.second - 1);
        adjacency_lists[current_edge.second - 1].push_back(current_edge.first - 1);

        confusion_matrix[current_edge.first - 1][current_edge.second - 1] = true;
        confusion_matrix[current_edge.second - 1][current_edge.first - 1] = true;

        confusion_matrix_bit_set[current_edge.first - 1][current_edge.second - 1] = true;
        confusion_matrix_bit_set[current_edge.second - 1][current_edge.first - 1] = true;
    }
    file.close();
    return CqlGraph(n, m, degrees, confusion_matrix, adjacency_lists, confusion_matrix_bit_set);
}

std::set<uint64_t> CqlGraph::getHeuristicMaxClique(const std::set<std::set<uint64_t>> &colorings) {
    std::size_t max_clique_upperbound = *std::max_element(degrees_.begin(), degrees_.end());

    for (const auto &strategy: nodes_ordering_strategies) {
        std::set<std::set<uint64_t>> current_coloring = colorGraph(strategy);
        std::set<uint64_t> current_clique = std::set<uint64_t>();
    }
    return std::set<uint64_t>();
}

std::set<uint64_t> CqlGraph::colorGraph(const CqlGraph::NodesOrderingStrategy &strategy) const {

    return std::set<uint64_t>();
}

std::vector<uint64_t> CqlGraph::orderVertices(const CqlGraph::NodesOrderingStrategy &strategy) const {
    std::vector<uint64_t> vertices(n_, 0);
    std::iota(vertices.begin(), vertices.end(), 0);

    switch (strategy) {
        case NodesOrderingStrategy::INDEX:
            return vertices;
        case NodesOrderingStrategy::LARGEST_DEGREE_FIRST:
            break;
        case NodesOrderingStrategy::SMALLEST_DEGREE_FIRST:
            break;
        case NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST:
            break;
        case NodesOrderingStrategy::RANDOM:
            break;
        case NodesOrderingStrategy::SATURATION_LARGEST_FIRST:
            break;
    }
}

std::vector<uint64_t> CqlGraph::orderVerticesLatestDegreeFirst(const std::vector<uint64_t> &vertices) const {
    std::sort(vertices.begin(), vertices.end(), [&](uint64_t i, uint64_t j) { return degrees_[i] > degrees_[j]; });
    return vertices;
}

std::vector<uint64_t> CqlGraph::orderVerticesSmallestDegreeFirst(const std::vector<uint64_t> &vertices) const {
    std::sort(vertices.begin(), vertices.end(), [&](uint64_t i, uint64_t j) { return degrees_[i] < degrees_[j]; });
    return vertices;
}

std::vector<uint64_t> CqlGraph::verticesSupport() const {
    std::vector<uint64_t> supports(n_, 0);
    for (std::size_t i = 0; i < n_; ++i) {
        for (const uint64_t neighbor: adjacency_lists_[i]) {
            supports[i] += degrees_[neighbor];
        }
    }
    return supports;
}

std::vector<uint64_t> CqlGraph::orderVerticesSmallestDegreeSupportFirst(const std::vector<uint64_t> &vertices) const {
    std::vector<uint64_t> mutable_degrees(degrees_);
    std::vector<uint64_t> support = verticesSupport();
    std::vector<uint64_t> ordered_vertices;

    auto set_function = [&](uint64_t i, uint64_t j) {
        return (mutable_degrees[i] < mutable_degrees[j]) || (support[i] < support[j]) || i < j;
    };

    std::set<int, decltype(set_function)> degree_support_index_queue(set_function);
    degree_support_index_queue.insert(vertices.begin(), vertices.end());

    while (!degree_support_index_queue.empty()) {
        uint64_t next_vertex = *degree_support_index_queue.begin();
        degree_support_index_queue.erase(degree_support_index_queue.begin());

        mutable_degrees[next_vertex] = 0;

        ordered_vertices.push_back(next_vertex);

        for (const uint64_t neighbor: adjacency_lists_[next_vertex]) {
            // check if vertex already in ordered vertexes
            if (mutable_degrees[neighbor] != 0) {
                degree_support_index_queue.erase(neighbor);
                mutable_degrees[neighbor]--;
                // Let's don't connect mutable_degrees and support. It is simpler
                support[neighbor] -= degrees_[next_vertex];
                degree_support_index_queue.insert(neighbor);
            }
        }
    }
    return ordered_vertices;
}

std::vector<uint64_t> CqlGraph::orderVerticesRandom(std::vector<uint64_t> vertices) const {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(vertices.begin(), vertices.end(), g);
    return vertices;
}

std::vector<uint64_t> CqlGraph::orderVerticesSaturationLargestFirst(std::vector<uint64_t> vertices) const {
    std::vector<uint64_t> ordered_vertices;
    std::vector<int64_t> saturation(n_, 0);
    std::map<uint64_t, std::set<uint64_t> > colors;

    auto set_function = [&](uint64_t i, uint64_t j) {
        return (saturation[i] > saturation[j]) || (degrees_[i] > degrees_[j]) || i < j;
    };

    std::set<int, decltype(set_function)> saturation_queue(set_function);
    saturation_queue.insert(vertices.begin(), vertices.end());

    while (saturation_queue.empty()) {
        uint64_t next_vertex = *saturation_queue.begin();
        saturation_queue.erase(saturation_queue.begin());

        saturation[next_vertex] = -1;

        ordered_vertices.push_back(next_vertex);
        for (const uint64_t neighbor: adjacency_lists_[next_vertex]) {
            // check if vertex already in ordered vertexes
            if (saturation[next_vertex] != -1) {
                saturation_queue.erase(neighbor);
                saturation[neighbor]++;
                saturation_queue.insert(neighbor);
            }
        }
    }
    return ordered_vertices;
}
