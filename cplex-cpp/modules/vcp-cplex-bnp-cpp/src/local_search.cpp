#include <bitset>
#include <utility>
#include <vector>
#include <include/shared.h>
#include "include/local_search.h"
#include <random>

LocalSearchExecutionContext::LocalSearchExecutionContext(const Bitset &current_solution,
                                                         const Graph &graph,
                                                         std::uint64_t init)
        : graph(graph), current_solution(current_solution) {
    non_solution_vertices = ~current_solution;
    current_solution_size = current_solution.count();
    tightness = calculateTightness(non_solution_vertices, init);
}

WISLocalSearchExecutionContext::WISLocalSearchExecutionContext(const Bitset &current_is,
                                                               const Graph &graph,
                                                               const std::vector<double> &weights) :
        LocalSearchExecutionContext(current_is, graph), weights(weights) {
    weights_diff = calculateWeightsDiff();
}

LocalSearchExecutionContext::LocalSearchExecutionContext(const Graph &graph,
                                                         Bitset current_solution,
                                                         std::size_t current_solution_size,
                                                         Bitset non_solution_vertices,
                                                         std::vector<uint64_t> tightness)
        : graph(graph),
          current_solution(current_solution),
          current_solution_size(current_solution_size),
          non_solution_vertices(non_solution_vertices),
          tightness(std::move(tightness)) {}


WISLocalSearchExecutionContext::WISLocalSearchExecutionContext(const WISLocalSearchExecutionContext &context) :
        LocalSearchExecutionContext(context.graph,
                                    context.current_solution,
                                    context.current_solution_size,
                                    context.non_solution_vertices,
                                    context.tightness),
        weights_diff(context.weights_diff),
        weights(context.weights) {}

WISLocalSearchExecutionContext &WISLocalSearchExecutionContext::operator=(const WISLocalSearchExecutionContext &other) {
    this->current_solution = other.current_solution;
    this->current_solution_size = other.current_solution_size;
    this->non_solution_vertices = other.non_solution_vertices;
    this->tightness = other.tightness;
    this->weights_diff = other.weights_diff;
    return *this;
}

CliqueLocalSearchExecutionContext::CliqueLocalSearchExecutionContext(const Bitset &clique,
                                                                     const Graph &graph)
        : LocalSearchExecutionContext(clique, graph, clique.count() - 1) {}

CliqueLocalSearchExecutionContext::CliqueLocalSearchExecutionContext(
        const CliqueLocalSearchExecutionContext &context) : LocalSearchExecutionContext(context.graph,
                                                                                        context.current_solution,
                                                                                        context.current_solution_size,
                                                                                        context.non_solution_vertices,
                                                                                        context.tightness) {

}

CliqueLocalSearchExecutionContext &CliqueLocalSearchExecutionContext::operator=(
        const CliqueLocalSearchExecutionContext &other) {
    this->current_solution = other.current_solution;
    this->current_solution_size = other.current_solution_size;
    this->non_solution_vertices = other.non_solution_vertices;
    this->tightness = other.tightness;
    return *this;
}


std::vector<double> WISLocalSearchExecutionContext::calculateWeightsDiff() {
    std::vector<double> result(weights);

    for (std::size_t i = 0; i < graph.n_; ++i) {
        Bitset neighbor_in_independent_set = graph.confusion_matrix_bit_set_[i] & current_solution;
        for (std::size_t j = 0; j < graph.n_; ++j) {
            result[i] -= weights[j] * neighbor_in_independent_set[j];
        }
    }
    return result;
}

//tightness - количетво соседей не принадлежащей решению вершины которые в решении
std::vector<uint64_t> LocalSearchExecutionContext::calculateTightness(Bitset possible_candidates,
                                                                      std::uint64_t init) const {
    std::vector<uint64_t> result(graph.n_, init);

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (possible_candidates[v]) {
            result[v] = ((graph.confusion_matrix_bit_set_[v]) & current_solution).count();
        }
    }
    return result;
}


void WISLocalSearchExecutionContext::localSearch() {

    uint64_t w1_swap = getNextW1SwapCandidate();
    Bitset used;
    while (w1_swap != UINT64_MAX) {
        if (used[w1_swap]) break;
        used.set(w1_swap, true);
        // delete neighbors and add new vertex
        updateSet(graph.confusion_matrix_bit_set_[w1_swap], w1_swap);
        w1_swap = getNextW1SwapCandidate();
    }

    std::map<uint64_t, Bitset> candidates_1_2_swap = build12SwapCandidatesSet(current_solution,
                                                                              tightness);
    for (auto it = candidates_1_2_swap.begin(); it != candidates_1_2_swap.end();) {
        auto x_to_candidates = *it;
        if (x_to_candidates.second.count() < 2) {
            ++it;
            continue;
        }

        std::pair<uint64_t, uint64_t> swap = findFirst12Swap(weights[x_to_candidates.first], x_to_candidates.second);

        if (swap.first == UINT64_MAX) {
            ++it;
            continue;
        }
        updateSetAndCandidates(x_to_candidates.first, swap, candidates_1_2_swap);

        // из кандидатов нужно удалить соседей тех вершин, которые мы вставили
        candidates_1_2_swap = build12SwapCandidatesSet(current_solution, tightness);
        it = candidates_1_2_swap.begin();
    }
}

void WISLocalSearchExecutionContext::perturb(std::size_t k) {
    std::random_device rd;
    std::mt19937 g(rd());

    for (std::size_t i = 0; i < k; ++i) {
        std::uniform_int_distribution<> distrib(0, (int) (graph.n_ - current_solution_size - 1));
        std::size_t vertex_to_insert = distrib(g);
        std::size_t index_vertex_to_insert = 0;
        std::size_t skipped = 0;

        for (; index_vertex_to_insert < graph.n_; ++index_vertex_to_insert) {
            if (current_solution[index_vertex_to_insert]) continue;

            if (skipped == vertex_to_insert) {
                break;
            }
            skipped++;
        }

        updateSet(graph.confusion_matrix_bit_set_[index_vertex_to_insert], index_vertex_to_insert);
    }
}

std::pair<uint64_t, uint64_t> WISLocalSearchExecutionContext::findFirst12Swap(double w_to_delete,
                                                                              const Bitset &candidates) const {
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!candidates[v]) continue;
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (!candidates[u]) continue;
            if (!graph.confusion_matrix_bit_set_[v][u] &&
                (lessThan(w_to_delete, weights[u] + weights[v]) || equals(w_to_delete, weights[u] + weights[v]))) {
                return {v, u};
            }
        }
    }
    return {UINT64_MAX, UINT64_MAX};
}


std::map<uint64_t, Bitset> WISLocalSearchExecutionContext::build12SwapCandidatesSet(Bitset current_solution,
                                                                                    const std::vector<uint64_t> &tightness) const {

    std::map<uint64_t, Bitset> candidates_per_vertex;
    Bitset free_vertices;
    Bitset non_solution_vertices = ~current_solution;

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (non_solution_vertices[v]) {
            // посчитать сколько соседей лежат в независемом множестве
            if (tightness[v] == 0) {
                free_vertices.set(v, true);
            } else if (tightness[v] == 1) {
                Bitset neighbor_in_independent_set = graph.confusion_matrix_bit_set_[v] & current_solution;
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
        for (std::pair<uint64_t, Bitset> entry: candidates_per_vertex) {
            entry.second |= free_vertices;
        }
    }

    return candidates_per_vertex;
}

uint64_t WISLocalSearchExecutionContext::getNextW1SwapCandidate() const {
    // (w, 1) swap - добаляем одну вершину и удаляем ее соседей
    // (только в том случае, если ее вес больше, чем вес всех ее соседей)
    uint64_t vertex_with_max_weights_diff = UINT64_MAX;
    for (std::size_t v = 0; v < graph.n_; ++v) {
        // рассматриваем только вершин не из решения и которые при добавлении не уменьшат вес решения
        if (current_solution[v] || std::round(weights_diff[v]) <= 0) continue;
        if (vertex_with_max_weights_diff == UINT64_MAX) {
            vertex_with_max_weights_diff = v;
        } else {
            vertex_with_max_weights_diff =
                    greaterThan(weights_diff[v], weights_diff[vertex_with_max_weights_diff]) ? v
                                                                                             : vertex_with_max_weights_diff;
        }
    }
    return vertex_with_max_weights_diff;
}

void WISLocalSearchExecutionContext::updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                                            std::map<uint64_t, Bitset> &candidates_1_2_swap) {
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

    current_solution_size = current_solution.count();

    for (const auto &neighbor: graph.adjacency_lists_[deleted]) {
        tightness[neighbor] -= 1 * (!current_solution[neighbor]);
    }

    for (const auto &neighbor: graph.adjacency_lists_[inserted.first]) {
//       weights_diff - разница между весом вершины и суммой весов соседей в вершине
//      Вставили вершину в решение -> значит нужно ее вес учесть в weights_diff для всех ее соседей
        weights_diff[neighbor] -= weights[inserted.first];
//      Вставили вершину в решение -> нужно увеличить на единицу тау у ее соседей
        tightness[neighbor] += 1 * (neighbor != deleted);
    }
    for (const auto &neighbor: graph.adjacency_lists_[inserted.second]) {
        weights_diff[neighbor] -= weights[inserted.second];
        tightness[neighbor] += 1 * (neighbor != deleted);
    }
//    Так как вершина удалена из решения -> она не должна влиять на weights_diff своих соседей
    for (const auto &neighbor: graph.adjacency_lists_[deleted]) {
        weights_diff[neighbor] += weights[deleted];
    }
}

void WISLocalSearchExecutionContext::updateSet(Bitset deleted, uint64_t inserted) {
    auto deleted_in_solution = current_solution & deleted;
    current_solution &= ~deleted;
    current_solution.set(inserted, true);
    current_solution_size = current_solution.count();

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!deleted_in_solution[v] || v == inserted) continue;
        for (const auto &neighbor: graph.adjacency_lists_[v]) {
            tightness[neighbor] -= 1 * (!deleted_in_solution[neighbor]);
            weights_diff[neighbor] += weights[v] * (!deleted_in_solution[neighbor]);
        }
        tightness[v] = 1;
    }

    tightness[inserted] = 0;

    for (const auto &neighbor: graph.adjacency_lists_[inserted]) {
        weights_diff[neighbor] -= weights[inserted];
        tightness[neighbor] += 1 * (!deleted_in_solution[neighbor]);
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


WeightWithColumn LocalSearchLauncher::localSearch(Bitset initial_solution,
                                                  const Graph &graph,
                                                  const std::vector<double> &weights) {
    WISLocalSearchExecutionContext current_solution(initial_solution, graph, weights);
    current_solution.localSearch();

    Bitset best_solution = current_solution.current_solution;
    double best_weight = current_solution.weight();
    uint64_t non_changed_iteration = 0;

    WISLocalSearchExecutionContext local_solution = current_solution;
    for (std::size_t iteration = 0; iteration < 100; ++iteration) {
        local_solution.perturb();
        local_solution.localSearch();
        //  acceptance
        double updated = local_solution.weight();
        double current = current_solution.weight();
        if (updated > current || non_changed_iteration > current_solution.current_solution_size) {
            current_solution = local_solution;
            if (updated > best_weight ||
                (updated == best_weight && best_solution.count() < local_solution.current_solution_size)) {
                best_solution = local_solution.current_solution;
                best_weight = updated;
            }
            non_changed_iteration = 0;
        } else {
            if (updated == current &&
                local_solution.current_solution_size > current_solution.current_solution_size) {
                current_solution = local_solution;
                if (updated > best_weight ||
                    (updated == best_weight && best_solution.count() < local_solution.current_solution_size)) {

                    best_solution = local_solution.current_solution;
                    best_weight = updated;
                }
                non_changed_iteration = 0;
            } else {
                non_changed_iteration++;
            }
        }
    }
    return {best_weight, best_solution};
}

std::pair<uint64_t, Bitset> LocalSearchLauncher::localSearch(Bitset initial_solution,
                                                             const Graph &graph) {
    CliqueLocalSearchExecutionContext current_solution(initial_solution, graph);
    current_solution.localSearch();

    Bitset best_solution = current_solution.current_solution;
    uint64_t best_size = current_solution.current_solution_size;
    uint64_t non_changed_iteration = 0;
    CliqueLocalSearchExecutionContext local_solution = current_solution;
    for (std::size_t iteration = 0; iteration < 10000; ++iteration) {
        local_solution.perturb();
        local_solution.localSearch();
        //  acceptance
        uint64_t updated = local_solution.current_solution_size;
        uint64_t current = current_solution.current_solution_size;
        if (updated > current || non_changed_iteration > current) {
            current_solution = local_solution;
            if (updated > best_size) {
                best_solution = local_solution.current_solution;
                best_size = updated;
            }
            non_changed_iteration = 0;
        } else {
            non_changed_iteration++;
        }
    }
    return {best_size, best_solution};
}

void CliqueLocalSearchExecutionContext::updateSetAndCandidates(uint64_t deleted, std::pair<uint64_t, uint64_t> inserted,
                                                               std::map<uint64_t, Bitset> &candidates_1_2_swap) {
    uint64_t inserted_connected_to_deleted =
            graph.confusion_matrix_bit_set_[deleted][inserted.first] +
            graph.confusion_matrix_bit_set_[deleted][inserted.second];

    tightness[deleted] = (current_solution_size - 1) + inserted_connected_to_deleted;
    current_solution.set(deleted, false);
    current_solution.set(inserted.first, true);
    current_solution.set(inserted.second, true);

    current_solution_size = current_solution.count();

    for (const auto &neighbor: graph.adjacency_lists_[deleted]) {
        tightness[neighbor] -= 1;
    }

    tightness[inserted.first] = (current_solution_size - 1);
    tightness[inserted.second] = (current_solution_size - 1);

    for (const auto &neighbor: graph.adjacency_lists_[inserted.first]) {
        tightness[neighbor] += 1 * (neighbor != inserted.second);
    }
    for (const auto &neighbor: graph.adjacency_lists_[inserted.second]) {
        tightness[neighbor] += 1 * (neighbor != inserted.first);
    }
}

std::map<uint64_t, Bitset> CliqueLocalSearchExecutionContext::build12SwapCandidatesSet(Bitset current_solution,
                                                                                       const std::vector<uint64_t> &tightness) const {

    std::map<uint64_t, Bitset> candidates_per_vertex;
    Bitset connected_to_clique;

    Bitset non_solution_vertices = ~current_solution;

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (non_solution_vertices[v]) {
            // посчитать сколько НЕсоседей лежат в клике
            if (tightness[v] == current_solution_size) {
                connected_to_clique.set(v, true);
            } else if (tightness[v] == current_solution_size - 1) {
                Bitset not_neighbor_in_clique = (~graph.confusion_matrix_bit_set_[v]) & current_solution;
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
        for (std::pair<uint64_t, Bitset> entry: candidates_per_vertex) {
            entry.second |= connected_to_clique;
        }
    }

    return candidates_per_vertex;
}

std::pair<uint64_t, uint64_t> CliqueLocalSearchExecutionContext::findFirst12Swap(Bitset candidates) const {
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!candidates[v]) continue;
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (!candidates[u]) continue;
            if (graph.confusion_matrix_bit_set_[v][u]) {
                return {v, u};
            }
        }
    }
    return {UINT64_MAX, UINT64_MAX};
}

void CliqueLocalSearchExecutionContext::perturb(size_t k) {
    std::random_device rd;
    std::mt19937 g(rd());

    for (std::size_t i = 0; i < k; ++i) {
        std::uniform_int_distribution<> distrib(0, (int) (graph.n_ - current_solution_size - 1));
        std::size_t vertex_to_insert = distrib(g);
        std::size_t index_vertex_to_insert = 0;
        std::size_t skipped = 0;
        for (; index_vertex_to_insert < graph.n_; ++index_vertex_to_insert) {
            if (current_solution[index_vertex_to_insert]) continue;

            if (skipped == vertex_to_insert) {
                break;
            }
            skipped++;
        }
        updateSet(~graph.confusion_matrix_bit_set_[index_vertex_to_insert], index_vertex_to_insert);
    }
}

void CliqueLocalSearchExecutionContext::updateSet(Bitset deleted, uint64_t inserted) {
    Bitset deleted_in_solution = current_solution & deleted;

    current_solution &= ~deleted;
    current_solution.set(inserted, true);
    current_solution_size = current_solution.count();

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!deleted_in_solution[v] || inserted == v) continue;
        // вершина была удалена - значит она связана со всеми старыми элементами клики
        // ее связанность с добавленной вершиной не учитывается, так как по определению здесь удалются только НЕ соседи
        for (const auto &neighbor: graph.adjacency_lists_[v]) {
            tightness[neighbor] -= 1 * (!deleted_in_solution[neighbor]);
        }
        tightness[v] = (current_solution_size - 1);
    }
    tightness[inserted] = (current_solution_size - 1);
    for (const auto &neighbor: graph.adjacency_lists_[inserted]) {
        tightness[neighbor] += 1;
    }
}

void CliqueLocalSearchExecutionContext::localSearch() {
    std::map<uint64_t, Bitset> candidates_1_2_swap = build12SwapCandidatesSet(current_solution,
                                                                              tightness);
    for (auto it = candidates_1_2_swap.begin(); it != candidates_1_2_swap.end();) {
        auto x_to_candidates = *it;
        if (x_to_candidates.second.count() < 2) {
            ++it;
            continue;
        }
        std::pair<uint64_t, uint64_t> swap = findFirst12Swap(x_to_candidates.second);

        if (swap.first == UINT64_MAX) {
            ++it;
            continue;
        }
        updateSetAndCandidates(x_to_candidates.first, swap, candidates_1_2_swap);
        candidates_1_2_swap = build12SwapCandidatesSet(current_solution, tightness);
        it = candidates_1_2_swap.begin();
    }
#ifdef CHECK_SOLUTION
    if (!graph.isClique(current_solution)) {
        std::cout << "this is not clique" << std::endl;
        throw std::runtime_error("this is not clique");
    }
#endif
}

std::pair<uint64_t, Bitset> LocalSearchLauncher::independentSetLocalSearch(Bitset initial_solution,
                                                                           const Graph &graph) {
    ISLocalSearchExecutionContext current_solution(initial_solution, graph);
    current_solution.localSearch();

    Bitset best_solution = current_solution.current_solution;
    uint64_t best_size = current_solution.current_solution_size;
    uint64_t non_changed_iteration = 0;

    ISLocalSearchExecutionContext local_solution = current_solution;
    for (std::size_t iteration = 0; iteration < 10; ++iteration) {
        local_solution.perturb();
        local_solution.localSearch();
        //  acceptance
        uint64_t updated = local_solution.current_solution_size;
        uint64_t current = current_solution.current_solution_size;
        if (updated > current || non_changed_iteration > current_solution.current_solution_size) {
            current_solution = local_solution;
            if (updated > best_size) {
                best_solution = local_solution.current_solution;
                best_size = updated;
            }
            non_changed_iteration = 0;
        } else {
            non_changed_iteration++;
        }
    }
    return {best_size, best_solution};
}

ISLocalSearchExecutionContext::ISLocalSearchExecutionContext(
        const Bitset &current_is, const Graph &graph) : LocalSearchExecutionContext(current_is, graph) {}

ISLocalSearchExecutionContext::ISLocalSearchExecutionContext(const ISLocalSearchExecutionContext &context) :
        LocalSearchExecutionContext(context.graph,
                                    context.current_solution,
                                    context.current_solution_size,
                                    context.non_solution_vertices,
                                    context.tightness) {}

ISLocalSearchExecutionContext &ISLocalSearchExecutionContext::operator=(const ISLocalSearchExecutionContext &other) {
    this->current_solution = other.current_solution;
    this->current_solution_size = other.current_solution_size;
    this->non_solution_vertices = other.non_solution_vertices;
    this->tightness = other.tightness;
    return *this;
}

void ISLocalSearchExecutionContext::updateSetAndCandidates(uint64_t deleted,
                                                           std::pair<uint64_t, uint64_t> inserted,
                                                           std::map<uint64_t, Bitset> &candidates_1_2_swap) {
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

    current_solution_size = current_solution.count();

    for (const auto &neighbor: graph.adjacency_lists_[deleted]) {
        tightness[neighbor] -= 1 * (!current_solution[neighbor]);
    }

    for (const auto &neighbor: graph.adjacency_lists_[inserted.first]) {
//      Вставили вершину в решение -> нужно увеличить на единицу тау у ее соседей
        tightness[neighbor] += 1 * (neighbor != deleted);
    }
    for (const auto &neighbor: graph.adjacency_lists_[inserted.second]) {
        tightness[neighbor] += 1 * (neighbor != deleted);
    }
}

std::map<uint64_t, Bitset> ISLocalSearchExecutionContext::build12SwapCandidatesSet(Bitset current_solution,
                                                                                   const std::vector<uint64_t> &tightness) const {
    std::map<uint64_t, Bitset> candidates_per_vertex;
    Bitset free_vertices;
    Bitset non_solution_vertices = ~current_solution;

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (non_solution_vertices[v]) {
            // посчитать сколько соседей лежат в независемом множестве
            if (tightness[v] == 0) {
                free_vertices.set(v, true);
            } else if (tightness[v] == 1) {
                Bitset neighbor_in_independent_set = graph.confusion_matrix_bit_set_[v] & current_solution;
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
        for (std::pair<uint64_t, Bitset> entry: candidates_per_vertex) {
            entry.second |= free_vertices;
        }
    }

    return candidates_per_vertex;
}

std::pair<uint64_t, uint64_t> ISLocalSearchExecutionContext::findFirst12Swap(Bitset candidates) const {
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!candidates[v]) continue;
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (!candidates[u]) continue;
            if (!graph.confusion_matrix_bit_set_[v][u]) {
                return {v, u};
            }
        }
    }
    return {UINT64_MAX, UINT64_MAX};
}

void ISLocalSearchExecutionContext::perturb(size_t k) {
    std::random_device rd;
    std::mt19937 g(rd());

    for (std::size_t i = 0; i < k; ++i) {
        std::uniform_int_distribution<> distrib(0, (int) (graph.n_ - current_solution_size - 1));
        std::size_t vertex_to_insert = distrib(g);
        std::size_t index_vertex_to_insert = 0;
        std::size_t skipped = 0;

        for (; index_vertex_to_insert < graph.n_; ++index_vertex_to_insert) {
            if (current_solution[index_vertex_to_insert]) continue;

            if (skipped == vertex_to_insert) {
                break;
            }
            skipped++;
        }

        updateSet(graph.confusion_matrix_bit_set_[index_vertex_to_insert], index_vertex_to_insert);
    }
}

void ISLocalSearchExecutionContext::updateSet(Bitset deleted, uint64_t inserted) {
    auto deleted_in_solution = current_solution & deleted;
    current_solution &= ~deleted;
    current_solution.set(inserted, true);
    current_solution_size = current_solution.count();

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!deleted_in_solution[v] || v == inserted) continue;
        for (const auto &neighbor: graph.adjacency_lists_[v]) {
            tightness[neighbor] -= 1 * (!deleted_in_solution[neighbor]);
        }
        tightness[v] = 1;
    }

    tightness[inserted] = 0;

    for (const auto &neighbor: graph.adjacency_lists_[inserted]) {
        tightness[neighbor] += 1 * (!deleted_in_solution[neighbor]);
    }
}

void ISLocalSearchExecutionContext::localSearch() {
    std::map<uint64_t, Bitset> candidates_1_2_swap = build12SwapCandidatesSet(current_solution,
                                                                              tightness);
    for (auto it = candidates_1_2_swap.begin(); it != candidates_1_2_swap.end();) {
        auto x_to_candidates = *it;
        if (x_to_candidates.second.count() < 2) {
            ++it;
            continue;
        }

        std::pair<uint64_t, uint64_t> swap = findFirst12Swap(x_to_candidates.second);

        if (swap.first == UINT64_MAX) {
            ++it;
            continue;
        }
        updateSetAndCandidates(x_to_candidates.first, swap, candidates_1_2_swap);

        // из кандидатов нужно удалить соседей тех вершин, которые мы вставили
        candidates_1_2_swap = build12SwapCandidatesSet(current_solution, tightness);
        it = candidates_1_2_swap.begin();
    }
}
