#define CHECK_SOLUTION

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

#include <cassert>

CqlGraph::CqlGraph(
        const std::size_t n,
        const std::size_t m,
        std::vector<std::set<uint64_t>> lists,
        std::vector<std::bitset<1024>> matrix_b) : n_(n),
                                                   m_(m),
                                                   adjacency_lists_(std::move(lists)),
                                                   confusion_matrix_bit_set_(std::move(matrix_b)) {}

CqlGraph CqlGraph::readGraph(const std::string &graphs_path, const std::string &graph_name) {
    std::size_t n = 0;
    std::size_t m = 0;
    std::vector<std::set<uint64_t>> adjacency_lists;
    // just big number
    std::vector<std::bitset<1024>> confusion_matrix_bit_set;

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
    confusion_matrix_bit_set.resize(n, std::bitset<1024>(false));

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
    return CqlGraph(n, m, adjacency_lists, confusion_matrix_bit_set);
}

std::set<uint64_t>
CqlGraph::getHeuristicMaxClique(const std::vector<uint64_t> &coloring, NodesOrderingStrategy cs) const {
    std::set<uint64_t> current_clique = std::set<uint64_t>();

    auto cmp = [&](uint64_t i, uint64_t j) {
        return (coloring[i] > coloring[j]) ||
               ((coloring[i] == coloring[j]) && (degree(i) < degree(j))) ||
               ((coloring[i] == coloring[j]) && (degree(i) == degree(j)) && i < j);
    };

    std::vector<uint64_t> vertices(n_);
    std::iota(vertices.begin(), vertices.end(), 0);
    std::sort(vertices.begin(), vertices.end(), cmp);

    std::bitset<1024> used;
    std::bitset<1024> candidates;

    candidates.set();

    while (true) {
        uint64_t best_candidate = 0;

        for (uint64_t v: vertices) {
            if (candidates[v] && !used[v]) {
                best_candidate = v;
                break;
            }
        }

        used[best_candidate] = true;
        current_clique.insert(best_candidate);

        candidates = candidates & confusion_matrix_bit_set_[best_candidate] & (~used);

        if (candidates.none()) {
            break;
        }
    }
#ifdef CHECK_SOLUTION
    if (!isClique(current_clique)) {
        std::cout << "this is not clique" << std::endl;
        throw std::runtime_error("this is not clique");
    }
#endif
    return current_clique;
}

std::bitset<1024> CqlGraph::getHeuristicMaxCliqueRecursive(const std::vector<uint64_t> &coloring,
                                                           NodesOrderingStrategy cs) const {
    std::bitset<1024> current_clique;

    std::bitset<1024> used;
    std::bitset<1024> candidates;

    candidates.set();

    uint64_t best_candidate = findCliqueBestCandidate(coloring, *this);
    uint64_t max_color = UINT64_MAX;

    while (true) {
        used[best_candidate] = true;
        current_clique.set(best_candidate, true);

        candidates = candidates & confusion_matrix_bit_set_[best_candidate] & (~used);

        if (candidates.none()) {
            break;
        }
        CqlGraph subgraph = buildSubgraph(candidates);

        auto curr_coloring_to_max_color = subgraph.colorGraph(cs);
        max_color = curr_coloring_to_max_color.second;

        if (max_color != 0) {
            best_candidate = findCliqueBestCandidate(curr_coloring_to_max_color.first, subgraph);
        } else {
            best_candidate = 0;
            while (!candidates.test(best_candidate)) best_candidate++;
        }
    }
#ifdef CHECK_SOLUTION
    if (!isClique(current_clique)) {
        std::cout << "this is not clique" << std::endl;
        throw std::runtime_error("this is not clique");
    }
#endif
    return current_clique;
}

// https://www.geeksforgeeks.org/graph-coloring-set-2-greedy-algorithm/
std::pair<std::vector<uint64_t>, uint64_t> CqlGraph::colorGraph(const NodesOrderingStrategy &strategy) const {
    std::vector<uint64_t> ordered_vertices = orderVertices(strategy);

    uint64_t max_color = 0;
    std::map<uint64_t, std::set<uint64_t>> coloring;
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

        max_color = std::max(min_color, max_color);
        colored_vertices[v] = min_color;
        coloring[min_color].insert(v);

        std::fill(used_colors.begin(), used_colors.end(), 0);
    }
#ifdef CHECK_SOLUTION
    if (!isColoringCorrect(colored_vertices)) {
        std::cout << "this coloring is not correct" << std::endl;
        throw std::runtime_error("this coloring is not correct");
    }
#endif
    return {colored_vertices, max_color};
}

std::vector<uint64_t> CqlGraph::orderVertices(const NodesOrderingStrategy &strategy) const {
    std::vector<uint64_t> vertices(n_, 0);
    std::iota(vertices.begin(), vertices.end(), 0);

    switch (strategy) {
        case NodesOrderingStrategy::INDEX:
            return vertices;
        case NodesOrderingStrategy::LARGEST_DEGREE_FIRST:
            return orderVerticesLatestDegreeFirst(vertices);
        case NodesOrderingStrategy::SMALLEST_DEGREE_FIRST:
            return orderVerticesSmallestDegreeFirst(vertices);
        case NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST:
            return orderVerticesSmallestDegreeSupportFirst(vertices);
        case NodesOrderingStrategy::RANDOM:
            return orderVerticesRandom(vertices);
        case NodesOrderingStrategy::SATURATION_SMALLEST_FIRST:
            return orderVerticesSaturationSmallestFirst(vertices);
    }
    return std::vector<uint64_t>();
}

std::vector<uint64_t> CqlGraph::orderVerticesLatestDegreeFirst(std::vector<uint64_t> vertices) const {
    std::sort(vertices.begin(), vertices.end(), [&](uint64_t i, uint64_t j) { return degree(i) > degree(j); });
    return vertices;
}

std::vector<uint64_t> CqlGraph::orderVerticesSmallestDegreeFirst(std::vector<uint64_t> vertices) const {
    std::sort(vertices.begin(), vertices.end(), [&](uint64_t i, uint64_t j) { return degree(i) < degree(j); });
    return vertices;
}

std::vector<uint64_t> CqlGraph::verticesSupport() const {
    std::vector<uint64_t> supports(n_, 0);
    for (std::size_t i = 0; i < n_; ++i) {
        for (const uint64_t neighbor: adjacency_lists_[i]) {
            supports[i] += degree(neighbor);
        }
    }
    return supports;
}

std::vector<uint64_t> CqlGraph::orderVerticesSmallestDegreeSupportFirst(std::vector<uint64_t> vertices) const {
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

std::vector<uint64_t> CqlGraph::orderVerticesRandom(std::vector<uint64_t> vertices) const {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(vertices.begin(), vertices.end(), g);
    return vertices;
}

std::vector<uint64_t> CqlGraph::orderVerticesSaturationSmallestFirst(std::vector<uint64_t> vertices) const {
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

bool CqlGraph::isColoringCorrect(std::vector<uint64_t> coloring) const {
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

bool CqlGraph::isClique(const std::set<uint64_t> &clique) const {
    for (const uint64_t v1: clique) {
        for (const uint64_t v2: clique) {
            if (v1 != v2 && !(confusion_matrix_bit_set_[v1][v2])) {
                return false;
            }
        }
    }
    return true;
}

bool CqlGraph::isClique(const std::bitset<1024> &clique) const {

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


std::bitset<1024> CqlGraph::localSearch(std::bitset<1024> &clique) const {
//  1-tight, if exactly 1 of its not neighbors lie in the solution.
// 2-improvement can be applied if the inserted adjacent vertices x, y are 1-tight vertices
//with common vertex v, which is to be removed from the solution Q.

// посчитать тау
// вычислить для него кадидатов для кажой вершины
// как учесть те вершины, которые соеденены со всеми? - в поиске в подграфе учитывать условие,
// что в старом графе хотя бы одна из них должны быть соеденена с вершиной v
// НО сказано как минимум одна! значит это не сломает решение
// несправедливо удаленная вершина может добавится в следующей итерации
    std::bitset<1024> possible_candidates = ~clique;
    std::vector<uint64_t> tightness = calculateTightness(clique, possible_candidates);
    std::map<uint64_t, std::bitset<1024>> candidates = buildCandidatesSet(clique, possible_candidates, tightness);

    for (std::pair<uint64_t, std::bitset<1024>> x_to_L_x: candidates) {
        if (x_to_L_x.second.count() < 2) {
//            std::cout << "too few candidates" << std::endl;
            continue;
        }

        CqlGraph subgraph = buildSubgraph(x_to_L_x.second);
        std::pair<uint64_t, uint64_t> swap = findFirstSwap(subgraph);

        if (swap.first == UINT64_MAX) {
//            std::cout << "Cannot found connected vertices" << std::endl;
            continue;
        }

//        std::cout << "Increase by 1!!!" << std::endl;
        updateCliqueAndCandidates(clique, tightness, candidates, x_to_L_x.first, swap);

#ifdef CHECK_SOLUTION
        if (!isClique(clique)) {
            std::cout << "this is not clique" << std::endl;
            throw std::runtime_error("this is not clique");
        }
#endif
    }
    return clique;
}


//tightness - количетво соседей не принадлежащей решению вершины которые в клике
std::vector<uint64_t> CqlGraph::calculateTightness(std::bitset<1024> clique,
                                                   std::bitset<1024> possible_candidates) const {
    std::vector<uint64_t> tightness(n_, 0);

    for (std::size_t v = 0; v < n_; ++v) {
        if (possible_candidates[v]) {
            tightness[v] = ((confusion_matrix_bit_set_[v]) & clique).count();
        }
    }
    return tightness;
}

std::map<uint64_t, std::bitset<1024>> CqlGraph::buildCandidatesSet(std::bitset<1024> clique,
                                                                   std::bitset<1024> possible_candidates,
                                                                   const std::vector<uint64_t> &tightness) const {
    std::map<uint64_t, std::bitset<1024>> candidates_per_vertex;
    std::bitset<1024> connected_to_clique;

    for (std::size_t v = 0; v < n_; ++v) {
        if (possible_candidates[v]) {
            // посчитать сколько НЕсоседей лежат в клике
            if (tightness[v] == clique.count()) {
                connected_to_clique.set(v, true);
            } else if (tightness[v] == clique.count() - 1) {
                std::bitset<1024> not_neighbor_in_clique = (~confusion_matrix_bit_set_[v]) & clique;
                if (not_neighbor_in_clique.count() == 1) {
                    //найти первую и единственную вершину
                    uint64_t u = 0;
                    while (!not_neighbor_in_clique.test(u)) u++;
                    candidates_per_vertex[u].set(v, true);
                }
            }
        }
    }
    if (connected_to_clique.count() > 0) {
        // потому что есть такие вешины, которые соединены со всей кликой
        // и их можно добавить в паре вместе с удалением любой вершины
        for (std::pair<uint64_t, std::bitset<1024>> entry: candidates_per_vertex) {
            entry.second |= connected_to_clique;
        }
    }

    return candidates_per_vertex;
}

CqlGraph CqlGraph::buildSubgraph(std::bitset<1024> vertices) const {
    std::vector<std::set<uint64_t>> lists(n_);
    std::vector<std::bitset<1024>> matrix_b(n_);

    for (std::size_t v = 0; v < n_; ++v) {
        if (!vertices[v]) {
            matrix_b[v] = std::bitset<1024>();
        } else {
            matrix_b[v] = confusion_matrix_bit_set_[v] & vertices;
        }
    }

    // может убрать ?
    for (std::size_t v = 0; v < n_; ++v) {
        for (std::size_t u = v; u < n_; ++u) {
            if (matrix_b[v][u]) {
                lists[v].insert(u);
                lists[u].insert(v);
            }
        }
    }

    return CqlGraph(n_, 0, lists, matrix_b);
}

std::pair<uint64_t, uint64_t> CqlGraph::findFirstSwap(const CqlGraph &graph) const {
    for (std::size_t v = 0; v < n_; ++v) {
        if (!graph.adjacency_lists_[v].empty()) {
            return {v, *graph.adjacency_lists_[v].begin()};
        }
    }
    return {UINT64_MAX, UINT64_MAX};
}

void CqlGraph::updateCliqueAndCandidates(std::bitset<1024> &clique,
                                         std::vector<uint64_t> &tightness,
                                         std::map<uint64_t, std::bitset<1024>> &candidates,
                                         uint64_t deleted,
                                         std::pair<uint64_t, uint64_t> &inserted) const {
    // обработать удаленную вершину - удалить из мапы и обновить tightness
    uint64_t inserted_connected_to_deleted =
            confusion_matrix_bit_set_[deleted][inserted.first] + confusion_matrix_bit_set_[deleted][inserted.second];

//   (clique.size() - 1) - old clique neighbors
    tightness[deleted] = (clique.size() - 1) + inserted_connected_to_deleted;

    clique.set(deleted, false);
    clique.set(inserted.first, true);
    clique.set(inserted.second, true);

    for (const auto &neighbor: adjacency_lists_[inserted.first]) {
        tightness[neighbor] += 1;
    }
    for (const auto &neighbor: adjacency_lists_[inserted.second]) {
        tightness[neighbor] += 1;
    }
    std::bitset<1024> connected_to_clique;

    for (std::size_t v = 0; v < n_; ++v) {
        if (~clique[v]) {
            // посчитать сколько НЕсоседей лежат в клике
            if (tightness[v] == clique.size()) {
                connected_to_clique.set(v, true);
            } else if (tightness[v] == clique.size() - 1) {
                std::bitset<1024> not_neighbor_in_clique = (~confusion_matrix_bit_set_[v]) & clique;
                if (not_neighbor_in_clique.count() == 1) {
                    //найти первую и единственную вершину
                    uint64_t u = 0;
                    while (not_neighbor_in_clique.test(u)) u++;
                    candidates[u].set(v, true);
                }
            }
        }
    }
    // потому что есть такие вешины, которые соединены со всей кликой
    // и их можно добавить в паре вместе с удалением любой вершины
    for (std::pair<uint64_t, std::bitset<1024>> entry: candidates) {
        entry.second |= connected_to_clique;
    }
}

uint64_t CqlGraph::findCliqueBestCandidate(const std::vector<uint64_t> &curr_coloring, CqlGraph graph) const {
    auto cmp = [&](uint64_t i, uint64_t j) {
        return (curr_coloring[i] > curr_coloring[j]) ||
               ((curr_coloring[i] == curr_coloring[j]) && (graph.degree(i) < graph.degree(j))) ||
               ((curr_coloring[i] == curr_coloring[j]) && (graph.degree(i) == graph.degree(j)) && i < j);
    };
    uint64_t best_candidate = 0;
    for (std::size_t v = 1; v < n_; ++v) {
        if (cmp(v, best_candidate)) {
            best_candidate = v;
        }
    }
    return best_candidate;
}

bool CqlGraph::isVerticesIndependent(std::set<uint64_t> &independent_set) const {
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
