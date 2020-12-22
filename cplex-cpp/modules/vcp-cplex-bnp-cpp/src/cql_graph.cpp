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

#include <cassert>

Graph::Graph(
        const std::size_t n,
        const std::size_t m,
        std::vector<std::set<uint64_t>> lists,
        std::vector<Bitset> matrix_b) : n_(n),
                                        m_(m),
                                        adjacency_lists_(std::move(lists)),
                                        confusion_matrix_bit_set_(std::move(matrix_b)) {
    vertices = std::vector<uint64_t>(n_, 0);
    std::iota(vertices.begin(), vertices.end(), 0);
}

Graph Graph::readGraph(const std::string &graphs_path, const std::string &graph_name) {
    std::size_t n = 0;
    std::size_t m = 0;
    std::vector<std::set<uint64_t>> adjacency_lists;
    std::vector<Bitset> confusion_matrix_bit_set;

    std::fstream file;
    file.open(graphs_path + "/" + graph_name, std::fstream::in | std::fstream::out | std::fstream::app);

    std::string type;
    std::string current_line;
    std::pair<uint64_t, uint64_t> current_edge;

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

    adjacency_lists.resize(n, std::set<uint64_t>());
    confusion_matrix_bit_set.resize(n, Bitset(false));

    while (std::getline(file, current_line)) {
        if (current_line[0] != 'e') {
            continue;
        }
        std::istringstream iss(current_line);
        iss >> type >> current_edge.first >> current_edge.second;

        adjacency_lists[current_edge.first - 1].insert(current_edge.second - 1);
        adjacency_lists[current_edge.second - 1].insert(current_edge.first - 1);

        confusion_matrix_bit_set[current_edge.first - 1][current_edge.second - 1] = true;
        confusion_matrix_bit_set[current_edge.second - 1][current_edge.first - 1] = true;
    }
    file.close();
    return Graph(n, m, adjacency_lists, confusion_matrix_bit_set);
}

IndependentSets Graph::colorWeightedGraph(const std::vector<double> &wights) const {
    std::vector<uint64_t> ordered_vertices = orderVerticesSaturationSmallestFirstWeighted(wights);

    std::map<uint64_t, Column> color_to_vertex_bitset;
    std::vector<uint64_t> colored_vertices(n_, UINT64_MAX);
    std::vector<bool> used_colors(n_, false);

    for (const uint64_t v: ordered_vertices) {
        for (const uint64_t neighbor: adjacency_lists_[v]) {
            if (colored_vertices[neighbor] != UINT64_MAX) {
                used_colors[colored_vertices[neighbor]] = true;
            }
        }

        std::size_t min_color = 0;
        while (used_colors[min_color]) min_color++;

        colored_vertices[v] = min_color;
        color_to_vertex_bitset[min_color].set(v, true);

        std::fill(used_colors.begin(), used_colors.end(), 0);
    }
#ifdef CHECK_SOLUTION
    if (!isColoringCorrect(colored_vertices)) {
        std::cout << "this color_to_vertex_bitset is not correct" << std::endl;
        throw std::runtime_error("this color_to_vertex_bitset is not correct");
    }
#endif
    IndependentSets sets;
    for (auto const &color: color_to_vertex_bitset) {
        sets.emplace(supplementSetsToMaximumForInclusion(color.second).second);
    }
    return sets;
}

// TODO investigate performance
IndependentSets Graph::getIndependentSetByColoring(const NodesOrderingStrategy &strategy) const {
    std::vector<uint64_t> ordered_vertices = orderVertices(strategy);

    std::map<uint64_t, Column> color_to_vertex_bitset;
    std::vector<uint64_t> colored_vertices(n_, UINT64_MAX);
    std::vector<bool> used_colors(n_, false);

    for (const uint64_t v: ordered_vertices) {
        for (const uint64_t neighbor: adjacency_lists_[v]) {
            if (colored_vertices[neighbor] != UINT64_MAX) {
                used_colors[colored_vertices[neighbor]] = true;
            }
        }

        std::size_t min_color = 0;
        while (used_colors[min_color]) min_color++;

        colored_vertices[v] = min_color;
        color_to_vertex_bitset[min_color].set(v, true);

        std::fill(used_colors.begin(), used_colors.end(), 0);
    }
#ifdef CHECK_SOLUTION
    if (!isColoringCorrect(colored_vertices)) {
        std::cout << "this color_to_vertex_bitset is not correct" << std::endl;
        throw std::runtime_error("this color_to_vertex_bitset is not correct");
    }
#endif
    IndependentSets sets;
    for (auto const &color: color_to_vertex_bitset) {
        sets.emplace(supplementSetsToMaximumForInclusion(color.second).second);
    }
    return sets;
}

std::set<WeightToVertices> Graph::getWeightedIndependentSet(const std::vector<double> &weights) const {
    std::set<WeightToVertices> result;
    IndependentSets independent_sets_by_coloring = colorWeightedGraph(weights);
    for (const auto &independent_set: independent_sets_by_coloring) {
        double weight = 0.0;
        for (std::size_t i = 0; i < n_; ++i) {
            weight += weights[i] * independent_set[i];
        }
        if (weight > 1.0) {
            result.emplace(std::make_pair(weight, independent_set));
        }
    }
    return result;
}

std::vector<uint64_t> Graph::verticesSupport() const {
    std::vector<uint64_t> supports(n_, 0);
    for (std::size_t i = 0; i < n_; ++i) {
        for (const uint64_t neighbor: adjacency_lists_[i]) {
            supports[i] += degree(neighbor);
        }
    }
    return supports;
}

std::vector<uint64_t> Graph::orderVerticesRandom() const {
    std::random_device rd;
    std::mt19937 g(rd());
    std::vector<uint64_t> local_vertices(n_, 0);
    std::iota(local_vertices.begin(), local_vertices.end(), 0);
    std::shuffle(local_vertices.begin(), local_vertices.end(), g);
    return local_vertices;
}

std::vector<uint64_t> Graph::orderVerticesLatestDegreeFirst() const {
    std::vector<uint64_t> local_vertices(n_, 0);
    std::iota(local_vertices.begin(), local_vertices.end(), 0);
    std::sort(local_vertices.begin(), local_vertices.end(),
              [&](uint64_t i, uint64_t j) { return degree(i) > degree(j); });
    return local_vertices;
}

std::vector<uint64_t> Graph::orderVerticesSmallestDegreeFirst() const {
    std::vector<uint64_t> local_vertices(n_, 0);
    std::iota(local_vertices.begin(), local_vertices.end(), 0);
    std::sort(local_vertices.begin(), local_vertices.end(),
              [&](uint64_t i, uint64_t j) { return degree(i) < degree(j); });
    return local_vertices;
}

std::vector<uint64_t> Graph::orderVerticesSaturationSmallestFirst() const {
    std::vector<uint64_t> ordered_vertices;
    std::vector<uint64_t> saturation(n_, 0);

    auto set_function = [&](uint64_t i, uint64_t j) {
        return (saturation[i] > saturation[j]) ||
               ((saturation[i] == saturation[j]) && (degree(i) > degree(j))) ||
               ((saturation[i] == saturation[j]) && (degree(i) == degree(j)) && i < j);
    };

    std::set<uint64_t, decltype(set_function)> saturation_queue(set_function);
    saturation_queue.insert(vertices.begin(), vertices.end());

    while (!saturation_queue.empty()) {
        uint64_t next_vertex = *saturation_queue.begin();
        saturation_queue.erase(saturation_queue.begin());

        saturation[next_vertex] = UINT64_MAX;

        ordered_vertices.push_back(next_vertex);
        for (const uint64_t neighbor: adjacency_lists_[next_vertex]) {
            // check if vertex already in ordered vertexes
            if (saturation[neighbor] != UINT64_MAX) {
                saturation_queue.erase(neighbor);
                saturation[neighbor]++;
                saturation_queue.insert(neighbor);
            }
        }
    }
    std::reverse(ordered_vertices.begin(), ordered_vertices.end());
    return ordered_vertices;
}

std::vector<uint64_t> Graph::orderVerticesSmallestDegreeSupportFirst() const {
    std::vector<uint64_t> mutable_degrees(n_, 0);
    std::vector<uint64_t> support = verticesSupport();
    std::vector<uint64_t> ordered_vertices;

    for (std::size_t i = 0; i < n_; ++i) {
        mutable_degrees[i] = degree(i);
    }

    auto set_function = [&](uint64_t i, uint64_t j) {
        return (mutable_degrees[i] < mutable_degrees[j]) ||
               ((mutable_degrees[i] == mutable_degrees[j]) && (support[i] < support[j])) ||
               ((mutable_degrees[i] == mutable_degrees[j]) && (support[i] == support[j]) && i < j);
    };

    std::set<uint64_t, decltype(set_function)> degree_support_index_queue(set_function);
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
                support[neighbor] -= degree(next_vertex);
                degree_support_index_queue.insert(neighbor);
            }
        }
    }
    return ordered_vertices;
}

std::vector<uint64_t> Graph::orderVertices(const NodesOrderingStrategy &strategy) const {
    switch (strategy) {
        case NodesOrderingStrategy::INDEX:
            return vertices;
        case NodesOrderingStrategy::LARGEST_DEGREE_FIRST:
            return orderVerticesLatestDegreeFirst();
        case NodesOrderingStrategy::SMALLEST_DEGREE_FIRST:
            return orderVerticesSmallestDegreeFirst();
        case NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST:
            return orderVerticesSmallestDegreeSupportFirst();
        case NodesOrderingStrategy::RANDOM:
            return orderVerticesRandom();
        case NodesOrderingStrategy::SATURATION_SMALLEST_FIRST:
            return orderVerticesSaturationSmallestFirst();
        case NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST_WEIGHTED:
            return vertices;
    }
    return vertices;
}

std::vector<uint64_t> Graph::orderVerticesSaturationSmallestFirstWeighted(const std::vector<double> &weights) const {
    std::vector<uint64_t> mutable_degrees(n_, 0);
    std::vector<uint64_t> support = verticesSupport();
    std::vector<uint64_t> ordered_vertices;

    for (std::size_t i = 0; i < n_; ++i) {
        mutable_degrees[i] = degree(i);
    }

    auto set_function = [&](uint64_t i, uint64_t j) {
        return (weights[i] > weights[j]) ||
               (weights[i] == weights[j] && mutable_degrees[i] < mutable_degrees[j]) ||
               (weights[i] == weights[j] && (mutable_degrees[i] == mutable_degrees[j]) && (support[i] < support[j])) ||
               (weights[i] == weights[j] && (mutable_degrees[i] == mutable_degrees[j]) && (support[i] == support[j]) &&
                i < j);
    };

    std::set<uint64_t, decltype(set_function)> degree_support_index_queue(set_function);
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
                support[neighbor] -= degree(next_vertex);
                degree_support_index_queue.insert(neighbor);
            }
        }
    }
    return ordered_vertices;
}


std::pair<bool, Column> Graph::supplementSetsToMaximumForInclusion(const Column &independent_set) const {
    Column supplemented = independent_set;
    Column candidates = independent_set;
    for (std::size_t i = 0; i < n_; ++i) {
        if (!independent_set[i]) continue;
        candidates |= confusion_matrix_bit_set_[i];
    }
    candidates = ~candidates;

    bool hasImprovements = false;
    for (std::size_t v = 0; v < n_; ++v) {
        if (candidates[v]) {
            hasImprovements = true;
            supplemented.set(v, true);
        }
    }
#ifdef CHECK_SOLUTION
    if (!isVerticesIndependent(supplemented)) {
        std::cout << "set is not independent" << std::endl;
        throw std::runtime_error("set is not independent");
    }
#endif
    return {hasImprovements, supplemented};
}

Graph Graph::buildComplementGraph() const {
    std::vector<std::set<uint64_t>> lists(n_);
    std::vector<Bitset> matrix_b(n_);

    for (std::size_t v = 0; v < n_; ++v) {
        matrix_b[v] = ~confusion_matrix_bit_set_[v];
        for (std::size_t u = n_; u < n_; ++u) {
            matrix_b[v].set(u, false);
        }
        matrix_b[v].set(v, false);
    }

    for (std::size_t v = 0; v < n_; ++v) {
        for (std::size_t u = v; u < n_; ++u) {
            if (matrix_b[v][u]) {
                lists[v].insert(u);
                lists[u].insert(v);
            }
        }
    }

    return Graph(n_, 0, lists, matrix_b);
}

bool Graph::isClique(const Bitset &clique) const {

    for (std::size_t v = 0; v < n_; ++v) {
        for (std::size_t u = 0; u < n_; ++u) {
            if (clique[u] && clique[v]) {
                if (u != v && !(confusion_matrix_bit_set_[u][v])) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool Graph::isColoringCorrect(std::vector<uint64_t> coloring) const {
    for (std::size_t i = 0; i < n_; ++i) {
        uint64_t current_color = coloring[i];
        for (const uint64_t neighbor: adjacency_lists_[i]) {
            if (coloring[neighbor] == current_color) {
                return false;
            }
        }
    }
    return true;
}

bool Graph::isVerticesIndependent(Column &independent_set) const {
    for (std::size_t v = 0; v < n_; ++v) {
        if (!independent_set[v]) continue;
        for (std::size_t u = 0; v < n_; ++v) {
            if (!independent_set[u]) continue;
            if (u != v && confusion_matrix_bit_set_[v][u]) {
                std::cout << u << "\t" << v << std::endl;
                return false;
            }
        }
    }
    return true;
}

bool Graph::isVerticesIndependent(std::set<uint64_t> &independent_set) const {
    for (auto v: independent_set) {
        for (auto u: independent_set) {
            if (u != v && confusion_matrix_bit_set_[v][u]) {
                std::cout << u << "\t" << v << std::endl;
                return false;
            }
        }
    }
    return true;
}