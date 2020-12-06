#include <bitset>
#include <utility>
#include <vector>
#include <include/shared.h>
#include "include/local_search.h"
#include <random>

LocalSearchExecutionContext::LocalSearchExecutionContext(const std::bitset<1024> &current_solution,
                                                         const CqlGraph &graph)
        : graph(graph), current_solution(current_solution) {
    non_solution_vertices = ~current_solution;
    tightness = calculateTightness(non_solution_vertices);
}

WISLocalSearchExecutionContext::WISLocalSearchExecutionContext(const std::bitset<1024> &current_is,
                                                               const CqlGraph &graph,
                                                               const std::vector<double> &weights) :
        LocalSearchExecutionContext(current_is, graph), weights(weights) {
    weights_diff = calculateWeightsDiff();
}

LocalSearchExecutionContext::LocalSearchExecutionContext(const CqlGraph &graph,
                                                         std::bitset<1024> current_solution,
                                                         std::bitset<1024> non_solution_vertices,
                                                         std::vector<uint64_t> tightness)
        : graph(graph),
          current_solution(current_solution),
          non_solution_vertices(non_solution_vertices),
          tightness(std::move(tightness)) {}


WISLocalSearchExecutionContext::WISLocalSearchExecutionContext(const WISLocalSearchExecutionContext &context) :
        LocalSearchExecutionContext(context.graph,
                                    context.current_solution,
                                    context.non_solution_vertices,
                                    context.tightness),
        weights_diff(context.weights_diff),
        weights(context.weights) {}

WISLocalSearchExecutionContext &WISLocalSearchExecutionContext::operator=(const WISLocalSearchExecutionContext &other) {
    this->current_solution = other.current_solution;
    this->non_solution_vertices = other.non_solution_vertices;
    this->tightness = other.tightness;
    this->weights_diff = other.weights_diff;
    return *this;
}

std::vector<double> WISLocalSearchExecutionContext::calculateWeightsDiff() {
    std::vector<double> result(weights);

    for (std::size_t i = 0; i < graph.n_; ++i) {
        std::bitset<1024> neighbor_in_independent_set = graph.confusion_matrix_bit_set_[i] & current_solution;
        for (std::size_t j = 0; j < graph.n_; ++j) {
            if (!neighbor_in_independent_set[j]) continue;
            result[i] -= weights[j];
        }
    }
    return result;
}

//tightness - количетво соседей не принадлежащей решению вершины которые в решении
std::vector<uint64_t> LocalSearchExecutionContext::calculateTightness(std::bitset<1024> possible_candidates) const {
    std::vector<uint64_t> result(graph.n_, 0);

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (possible_candidates[v]) {
            result[v] = ((graph.confusion_matrix_bit_set_[v]) & current_solution).count();
        }
    }
    return result;
}


void WISLocalSearchExecutionContext::localSearch() {
    std::map<uint64_t, std::bitset<1024>> candidates_1_2_swap = build12SwapCandidatesSet(current_solution,
                                                                                         non_solution_vertices,
                                                                                         tightness);
    for (auto it = candidates_1_2_swap.begin(); it != candidates_1_2_swap.end(); ++it) {
        auto x_to_candidates = *it;
        if (x_to_candidates.second.count() < 2) {
            continue;
        }

        std::pair<uint64_t, uint64_t> swap = findFirst12Swap(weights[x_to_candidates.first], x_to_candidates.second);

        if (swap.first == UINT64_MAX) {
            continue;
        }
//        std::cout << "Found (1,2) swap: " << x_to_candidates.first << " -> (" << swap.first << "," << swap.second << ")"<< std::endl;
        updateSetAndCandidates(x_to_candidates.first, swap, candidates_1_2_swap);
        it = candidates_1_2_swap.begin();
    }
    auto set_function = [&](uint64_t i, uint64_t j) {
        return (weights_diff[i] > weights_diff[j]) || (weights_diff[i] == weights_diff[j] && i < j);
    };

    std::set<uint64_t, decltype(set_function)> w1_swap_candidates(set_function);

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (current_solution[v] || weights_diff[v] <= 0) continue;
        w1_swap_candidates.insert(v);
    }
    std::bitset<1024> deleted_neighbors;

    for (auto v: w1_swap_candidates) {
        deleted_neighbors &= graph.confusion_matrix_bit_set_[v];
        if ((deleted_neighbors & graph.confusion_matrix_bit_set_[v]).count() != 0) {
            continue;
        }
        updateSetAndCandidates(graph.confusion_matrix_bit_set_[v], v, candidates_1_2_swap);
        deleted_neighbors |= graph.confusion_matrix_bit_set_[v];
    }
}

std::bitset<1024> WISLocalSearchExecutionContext::perturb(std::size_t k) {
    std::random_device rd;
    std::mt19937 g(rd());

    std::uniform_int_distribution<> distrib(0, (int) (graph.n_ - current_solution.count() - 1));
    std::bitset<1024> result = current_solution;

    for (std::size_t i = 0; i < k; ++i) {
        std::size_t vertex_to_insert = distrib(g);
        std::size_t index_vertex_to_insert = 0;
        std::size_t skipped = 0;

        for (std::size_t v = 0; v < graph.n_; ++v, ++index_vertex_to_insert) {
            if (current_solution[v]) continue;

            if (skipped == vertex_to_insert) {
                break;
            }
            skipped++;
        }

        updateSet(graph.confusion_matrix_bit_set_[index_vertex_to_insert], index_vertex_to_insert);
    }
    return result;
}

std::pair<uint64_t, uint64_t> WISLocalSearchExecutionContext::findFirst12Swap(double w_to_delete,
                                                                              const std::bitset<1024> &candidates) const {
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!candidates[v]) continue;
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (!candidates[u]) continue;
            if (!graph.confusion_matrix_bit_set_[v][u] && w_to_delete <= weights[u] + weights[v]) {
                return {v, u};
            }
        }
    }
    return {UINT64_MAX, UINT64_MAX};
}


std::map<uint64_t, std::bitset<1024>> WISLocalSearchExecutionContext::build12SwapCandidatesSet(
        std::bitset<1024> current_solution,
        std::bitset<1024> non_solution_vertices,
        const std::vector<uint64_t> &tightness) const {

    std::map<uint64_t, std::bitset<1024>> candidates_per_vertex;
    std::bitset<1024> free_vertices;
    non_solution_vertices = ~current_solution;

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (non_solution_vertices[v]) {
            // посчитать сколько соседей лежат в независемом множестве
            if (tightness[v] == 0) {
                free_vertices.set(v, true);
            } else if (tightness[v] == 1) {
                std::bitset<1024> neighbor_in_independent_set = graph.confusion_matrix_bit_set_[v] & current_solution;
                if (neighbor_in_independent_set.count() == 1) {
                    //найти первую и единственную вершину
                    uint64_t u = 0;
                    while (!neighbor_in_independent_set.test(u)) u++;
                    candidates_per_vertex[u].set(v, true);
                }
            }
        }
    }
    if (free_vertices.count() > 0) {
        // потому что есть такие вешины, которые не соединены ни с какой вершинов в множестве
        // и их можно добавить в паре вместе с удалением любой вершины
        for (std::pair<uint64_t, std::bitset<1024>> entry: candidates_per_vertex) {
            entry.second |= free_vertices;
        }
    }

    return candidates_per_vertex;
}

void WISLocalSearchExecutionContext::updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                                            std::map<uint64_t, std::bitset<1024>> &candidates_1_2_swap) {
    uint64_t inserted_connected_to_deleted =
            graph.confusion_matrix_bit_set_[deleted][inserted.first] +
            graph.confusion_matrix_bit_set_[deleted][inserted.second];
// У удаленной вершины из соседей в решении могут быть только те, которые мы вставили
    tightness[deleted] = inserted_connected_to_deleted;
    tightness[inserted.first] = 0;
    tightness[inserted.second] = 0;

    current_solution.set(deleted, false);
    current_solution.set(inserted.first, true);
    current_solution.set(inserted.second, true);

    for (const auto &neighbor: graph.adjacency_lists_[inserted.first]) {
//       weights_diff - разница между весом вершины и суммой весов соседей в вершине
//      Вставили вершину в решение -> значит нужно ее вес учесть в weights_diff для всех ее соседей
        weights_diff[neighbor] -= weights[inserted.first];
//      Вставили вершину в решение -> нужно увеличить на единицу тау у ее соседей
        tightness[neighbor] += 1;
    }
    for (const auto &neighbor: graph.adjacency_lists_[inserted.second]) {
        weights_diff[neighbor] -= weights[inserted.second];
        tightness[neighbor] += 1;
    }
//    Так как вершина удалена из решения -> она не должна влиять на weights_diff своих соседей
    for (const auto &neighbor: graph.adjacency_lists_[deleted]) {
        weights_diff[neighbor] += weights[deleted];
    }

    // из кандидатов нужно удалить соседей тех вершин, которые мы вставили
    candidates_1_2_swap = build12SwapCandidatesSet(current_solution, ~current_solution, tightness);
}

void WISLocalSearchExecutionContext::updateSetAndCandidates(std::bitset<1024> deleted, uint64_t inserted,
                                                            std::map<uint64_t, std::bitset<1024>> &candidates_1_2_swap) {
    current_solution &= ~deleted;
    current_solution.set(inserted, true);

    tightness[inserted] = 0;
    for (const auto &neighbor: graph.adjacency_lists_[inserted]) {
        weights_diff[neighbor] -= weights[inserted];
        tightness[neighbor] += 1;
    }

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!deleted[v]) continue;
//      Происходит вставка одной вершины и удаление всех ее соседей -> у удаленных ершин только один сосед в решении
        tightness[v] = 1;
        for (const auto &neighbor: graph.adjacency_lists_[v]) {
            weights_diff[neighbor] += weights[v];
        }
    }
}

void WISLocalSearchExecutionContext::updateSet(std::bitset<1024> deleted, uint64_t inserted) {
    current_solution &= ~deleted;
    current_solution.set(inserted, true);

    tightness[inserted] = 0;
    for (const auto &neighbor: graph.adjacency_lists_[inserted]) {
        weights_diff[neighbor] -= weights[inserted];
        tightness[neighbor] += 1;
    }

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!deleted[v]) continue;
        tightness[v] = 1;
        for (const auto &neighbor: graph.adjacency_lists_[v]) {
            weights_diff[neighbor] += weights[v];
        }
    }
}

double WISLocalSearchExecutionContext::weight() {
    double weight = 0.0;
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!current_solution[v]) continue;
        weight += weights[v];
    }
    return weight;
}


std::pair<double, std::bitset<1024>> LocalSearchLauncher::localSearch(std::bitset<1024> initial_solution,
                                                                      const CqlGraph &graph,
                                                                      const std::vector<double> &weights,
                                                                      std::size_t max_iteration) {
    WISLocalSearchExecutionContext current_solution(initial_solution, graph, weights);
    current_solution.localSearch();

    std::bitset<1024> best_solution = current_solution.current_solution;
    double best_weight = current_solution.weight();

    WISLocalSearchExecutionContext local_solution = current_solution;
    for (std::size_t iteration = 0; iteration < 50; ++iteration) {
        local_solution.perturb();
        local_solution.localSearch();
        //  acceptance
        double updated = local_solution.weight();
        double current = current_solution.weight();
        if (updated > current) {
            current_solution = local_solution;
            if (updated > best_weight ||
                (updated == best_weight && best_solution.count() < local_solution.current_solution.count())) {
                best_solution = local_solution.current_solution;
                best_weight = updated;
            }
        } else {
            if (updated == current &&
                local_solution.current_solution.count() > current_solution.current_solution.count()) {
                current_solution = local_solution;
                if (updated > best_weight ||
                    (updated == best_weight && best_solution.count() < local_solution.current_solution.count())) {

                    best_solution = local_solution.current_solution;
                    best_weight = updated;
                }
            }
        }
    }
    return {best_weight, best_solution};
}
