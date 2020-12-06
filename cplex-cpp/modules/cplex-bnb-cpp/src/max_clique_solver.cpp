#include <algorithm>
#include <thread>
#include <random>
#include "include/max_clique_solver.h"

//#define CHECK_SOLUTION

std::set<std::set<uint64_t>> max_clique_solver::buildAdjacencyConstraints(const CqlGraph &graph) {
    std::set<std::set<uint64_t>> result;

//  add constraints for every i,j: x_i + x_j <= 1 if E(i,j) = 0
    for (std::size_t v = 0; v < graph.n_; ++v) {
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (!graph.confusion_matrix_bit_set_[v][u]) {
                std::set<uint64_t> constraint;
                constraint.insert(u);
                constraint.insert(v);
                result.insert(constraint);
            }
        }
    }

    //  add constraints for every v_i .. v_j: x_i + ... +  x_j <= 1 if E(v_i .. v_j) = 0
    std::vector<uint64_t> ordered = graph.orderVertices(NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST);

    auto vertices_order = [&](uint64_t i, uint64_t j) {
        return (graph.degree(i) < graph.degree(j)) || (graph.degree(i) == graph.degree(j) && i < j);
    };

    std::vector<std::vector<uint64_t>> independent_vertices_per_node(graph.n_);

    for (uint32_t v = 0; v < graph.n_; ++v) {
        for (uint32_t u = v + 1; u < graph.n_; ++u) {
            if (u != v && !graph.confusion_matrix_bit_set_[u][v]) {
                independent_vertices_per_node[u].push_back(v);
                independent_vertices_per_node[v].push_back(u);
            }
        }
    }

    for (uint32_t v = 0; v < graph.n_; ++v) {
        std::sort(independent_vertices_per_node[v].begin(), independent_vertices_per_node[v].end(), vertices_order);
    }

    for (uint64_t v: ordered) {
        std::set<uint64_t> constraint;

        constraint.emplace(v);
        std::vector<uint64_t> candidates = independent_vertices_per_node[v];

        while (!candidates.empty()) {
            uint64_t inserted_vertex = *candidates.begin();

            constraint.emplace(*candidates.begin());
            std::vector<uint64_t> new_candidates;
            std::set_intersection(candidates.begin(), candidates.end(),
                                  independent_vertices_per_node[inserted_vertex].begin(),
                                  independent_vertices_per_node[inserted_vertex].end(),
                                  inserter(new_candidates, new_candidates.begin()));
            candidates = new_candidates;
        }
        if (constraint.size() > 2) {
            result.emplace(constraint);
        }
    }
    return result;
}


std::set<std::set<uint64_t>> max_clique_solver::buildColoringConstraints(const CqlGraph &graph,
                                                                         const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring) {
    std::set<std::set<uint64_t>> result;
    std::map<uint64_t, std::set<uint64_t >> vertices_by_color;
    for (const auto &type_to_coloring : coloring) {
        vertices_by_color.clear();

        for (std::size_t v = 0; v < graph.n_; ++v) {
            vertices_by_color[type_to_coloring.second[v]].insert(v);
        }

        for (const auto &independent_set: vertices_by_color) {
            if (independent_set.second.size() <= 2)
                continue;

            // it really helps
            improveIndependentSet(graph, independent_set.second, result);
        }
    }
    return result;
}


void max_clique_solver::improveIndependentSet(const CqlGraph &graph,
                                              const std::set<uint64_t> &independent_set,
                                              std::set<std::set<uint64_t>> &result) {
    std::set<uint64_t> can_be_added;
    std::bitset<1024> independent_set_bit;

    for (auto v: independent_set) {
        independent_set_bit.set(v, true);
    }

    for (auto v: independent_set) {
        independent_set_bit |= graph.confusion_matrix_bit_set_[v];
    }

    bool hasImprovements = !independent_set_bit.all();
    if (hasImprovements) {
        std::bitset<1024> candidates_to_independent_set = ~independent_set_bit;
        CqlGraph subgraph = graph.buildSubgraph(candidates_to_independent_set);
        auto coloring = subgraph.colorGraph(NodesOrderingStrategy::SMALLEST_DEGREE_SUPPORT_FIRST);
        std::map<uint64_t, std::set<uint64_t >> vertices_by_color;

        for (std::size_t v = 0; v < graph.n_; ++v) {
            if (candidates_to_independent_set[v]) {
                vertices_by_color[coloring.first[v]].insert(v);
            }
        }
        for (const auto &nested_independent_set: vertices_by_color) {
            std::set<uint64_t> constraint(nested_independent_set.second);
            constraint.insert(independent_set.begin(), independent_set.end());
#ifdef CHECK_SOLUTION
            if (!graph.isVerticesIndependent(constraint)) {
                std::cout << "set is not independent" << std::endl;
                throw std::runtime_error("set is not independent");
            }
#endif
            result.insert(constraint);
        }
    } else {
        result.emplace(std::set<uint64_t>{
                independent_set.begin(),
                independent_set.end()
        });
    }
}

void max_clique_solver::improveIndependentSetByOne(const CqlGraph &graph,
                                                   const std::set<uint64_t> &independent_set,
                                                   std::set<std::set<uint64_t>> &result) {
    std::set<uint64_t> can_be_added;
    std::bitset<1024> independent_set_bit;

    for (auto v: independent_set) {
        independent_set_bit.set(v, true);
    }

    for (auto v: independent_set) {
        independent_set_bit |= graph.confusion_matrix_bit_set_[v];
    }
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!independent_set_bit[v]) {
            can_be_added.insert(v);
        }
    }
    for (auto v: can_be_added) {
        std::set<uint64_t> constraint(independent_set);
        constraint.insert(v);
#ifdef CHECK_SOLUTION
        std::cout<<"CHECK"<<std::endl;
            if (!graph.isVerticesIndependent(constraint)) {
                std::cout << "set is not independent" << std::endl;
                throw std::runtime_error("set is not independent");
            }
#endif
        result.insert(constraint);
    }

    if (can_be_added.empty()) {
        result.emplace(std::set<uint64_t>{
                independent_set.begin(),
                independent_set.end()
        });
    }
}

CplexModel max_clique_solver::init_cplex_model(const CqlGraph &graph,
                                               const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring) {
    CplexModel cplex_solver(graph.n_);

    std::set<std::set<uint64_t>> constraints;
    std::set<std::set<uint64_t>> adjacencyConstraints = buildAdjacencyConstraints(graph);
    std::set<std::set<uint64_t>> coloringConstraints = buildColoringConstraints(graph, coloring);

    constraints.insert(adjacencyConstraints.begin(), adjacencyConstraints.end());
//    std::cout << "before " << constraints.size() << "\t";
    constraints.insert(coloringConstraints.begin(), coloringConstraints.end());
//    std::cout << "after " << constraints.size() << std::endl;

    cplex_solver.addConstraints(constraints, 0, 1);
    return cplex_solver;
}

std::map<std::string, std::string> max_clique_solver::solve(const CqlGraph &graph, const Strategy &calc_strategy) {
    std::map<std::string, std::string> log;
    std::map<NodesOrderingStrategy, std::vector<uint64_t>> coloring_by_strategy;

    steady_clock::time_point begin = steady_clock::now();

    std::bitset<1024> best_clique = getBestMaxClique(graph, coloring_by_strategy);
    CplexModel cplex_solver = init_cplex_model(graph, coloring_by_strategy);

    auto heuristic_time = steady_clock::now() - begin;

    ExecutionContext bnb_context(best_clique.count(), hours(1), graph);
    try {
        bnb_context.start(cplex_solver);
    } catch (const std::exception &e) {
        std::cout << "HERE" << e.what() << std::endl;
    }

    log["heuristic_time (sec)"] = std::to_string(duration_cast<seconds>(heuristic_time).count());
    log["heuristic_time (ms)"] = std::to_string(duration_cast<milliseconds>(heuristic_time).count());
    log["heuristic_result"] = std::to_string(best_clique.count());
    log["cplex time (sec)"] = std::to_string(duration_cast<seconds>(bnb_context.metrics.total_execution_time).count());
    log["cplex time (ms)"] = std::to_string(
            duration_cast<milliseconds>(bnb_context.metrics.total_execution_time).count());
    log["result"] = std::to_string(bnb_context.optimal_solution.size);
    log["max_depth"] = std::to_string(bnb_context.metrics.max_depth);
    log["branches_num"] = std::to_string(bnb_context.metrics.branches_num);
    log["discarded_branches_num"] = std::to_string(bnb_context.metrics.discarded_branches_num);
    log["average_float_cplex_time"] =
            std::to_string(duration_cast<milliseconds>(bnb_context.metrics.average_float_cplex_time).count());
    log["float_cplex_time"] =
            std::to_string(duration_cast<milliseconds>(bnb_context.metrics.float_cplex_time).count());
    log["time (sec)"] = std::to_string(
            duration_cast<seconds>(bnb_context.metrics.total_execution_time + heuristic_time).count());
    log["timeout"] = std::to_string(bnb_context.timer.is_time_over);
    return log;
}

std::bitset<1024> max_clique_solver::getBestMaxClique(const CqlGraph &graph,
                                                      std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring_by_strategy) {
    std::bitset<1024> best_clique;
    std::size_t max_possible_clique = graph.maxDegree() + 1;

    for (auto coloring_strategy: nodes_ordering_strategies) {

        if (best_clique.size() == max_possible_clique) {
            break;
        }

        auto coloring_to_max_color = graph.colorGraph(coloring_strategy);
        coloring_by_strategy[coloring_strategy] = coloring_to_max_color.first;
        std::bitset<1024> current_clique = graph.getHeuristicMaxCliqueRecursive(coloring_to_max_color.first,
                                                                                coloring_strategy);
        if (current_clique.count() > best_clique.count()) {
            best_clique = current_clique;
        }
    }
    best_clique = graph.localSearch(best_clique);
    return best_clique;
}

double max_clique_solver::ExecutionContext::roundWithEpsilon(double objective_function_value,
                                                             double eps) {
    if (isNumberInteger(objective_function_value)) {
        return objective_function_value;
    }

    double up = std::ceil(objective_function_value);
    double down = std::floor(objective_function_value);

    if (objective_function_value + eps >= up) {
        return up;
    } else {
        return down;
    }
}

bool max_clique_solver::ExecutionContext::isResultInteger(const std::vector<double> &result) {
    for (const double &element: result) {
        if (!(isNumberInteger(element) || isNumberCloseToInteger(element))) {
            return false;
        }
    }
    return true;
}

uint64_t max_clique_solver::ExecutionContext::branchingFindNearestToOne(const FloatSolution &solution) {
    std::pair<uint64_t, double> nearest_to_one = {0, 1.0};
    for (uint64_t i = 0; i < solution.values.size(); ++i) {
        if (isNumberInteger(solution.values[i])) {
            continue;
        }
        double diff = 1.0 - solution.values[i];
        if (diff < nearest_to_one.second) {
            nearest_to_one.first = i;
            nearest_to_one.second = diff;
        }
    }
    return nearest_to_one.first;
}

uint64_t max_clique_solver::ExecutionContext::branchingFindNearestToZero(const FloatSolution &solution) {
    std::pair<uint64_t, double> nearest_to_zero = {0, 1.0};
    for (uint64_t i = 0; i < solution.values.size(); ++i) {
        if (isNumberInteger(solution.values[i])) {
            continue;
        }
        if (solution.values[i] < nearest_to_zero.second) {
            nearest_to_zero.first = i;
            nearest_to_zero.second = solution.values[i];
        }
    }
    return nearest_to_zero.first;
}

uint64_t max_clique_solver::ExecutionContext::branchingFindNearestToInteger(const FloatSolution &solution) {
    std::pair<uint64_t, double> nearest_to_one = {0, 1.0};
    for (uint64_t i = 0; i < solution.values.size(); ++i) {
        if (isNumberInteger(solution.values[i])) {
            continue;
        }
        double diff = min(1.0 - solution.values[i], solution.values[i]);
        if (diff < nearest_to_one.second) {
            nearest_to_one.first = i;
            nearest_to_one.second = diff;
        }
    }
    return nearest_to_one.first;
}

uint64_t max_clique_solver::ExecutionContext::branchingFindNearestToMiddle(const FloatSolution &solution) {
    std::pair<uint64_t, double> nearest_to_one = {0, 1.0};
    for (uint64_t i = 0; i < solution.values.size(); ++i) {
        if (isNumberInteger(solution.values[i])) {
            continue;
        }
        double diff = std::fabs(0.5 - solution.values[i]);
        if (diff < nearest_to_one.second) {
            nearest_to_one.first = i;
            nearest_to_one.second = diff;
        }
        if (diff == 0.5) {
            break;
        }
    }
    return nearest_to_one.first;
}

max_clique_solver::ExecutionContext::ExecutionContext(std::size_t heuristic_size,
                                                      const steady_clock::duration &time_to_execute,
                                                      const CqlGraph &graph) :
        optimal_solution({static_cast<double>(heuristic_size), std::vector<double>()}),
        timer(time_to_execute),
        graph(graph) {}

void max_clique_solver::ExecutionContext::start(CplexModel &model) {
    steady_clock::time_point begin = steady_clock::now();
    branchAndBound(model);
    metrics.total_execution_time = steady_clock::now() - begin;
    metrics.average_float_cplex_time = metrics.float_cplex_time / metrics.branches_num;
}

void max_clique_solver::ExecutionContext::branchAndBound(CplexModel &current_model) {
    if (timer.is_time_over) {
        return;
    }

    metrics.onStartBranch();
    metrics.onCplexFloatSolveStart();
    FloatSolution current_solution = current_model.getFloatSolution();
    metrics.onCplexFloatSolveFinish();

    double current_solution_size = roundWithEpsilon(current_solution.size);

    if (this->optimal_solution.size > current_solution_size) {
        metrics.onDiscardedBranch();
        return;
    }

    if (isResultInteger(current_solution.values)) {
        this->optimal_solution = current_solution;
        return;
    }

    uint64_t branching_var = branchingFindNearestToInteger(current_solution);

    IloRange down_constraint = current_model.addEqualityConstraintToVariable(branching_var, lower_bound);
    branchAndBound(current_model);
    current_model.deleteConstraint(down_constraint);

    IloRange up_constraint = current_model.addEqualityConstraintToVariable(branching_var, upper_bound);
    branchAndBound(current_model);
    current_model.deleteConstraint(up_constraint);

    metrics.onFinishBranch();
}

void max_clique_solver::ExecutionContext::branchAndBound(CplexModel &current_model,
                                                         const FloatSolution &current_solution) {
    if (timer.is_time_over) {
        return;
    }

    metrics.onStartBranch();

    double current_solution_size = roundWithEpsilon(current_solution.size);

    if (this->optimal_solution.size > current_solution_size) {
        metrics.onDiscardedBranch();
        return;
    }

    if (isResultInteger(current_solution.values)) {
        this->optimal_solution = current_solution;
        return;
    }

    uint64_t branching_var = branchingFindNearestToInteger(current_solution);

    IloRange down_constraint = current_model.addEqualityConstraintToVariable(branching_var, lower_bound);
    metrics.onCplexFloatSolveStart();
    FloatSolution down_solution = current_model.getFloatSolution();
    metrics.onCplexFloatSolveFinish();
    current_model.deleteConstraint(down_constraint);

    IloRange up_constraint = current_model.addEqualityConstraintToVariable(branching_var, upper_bound);
    metrics.onCplexFloatSolveStart();
    FloatSolution up_solution = current_model.getFloatSolution();
    metrics.onCplexFloatSolveFinish();
    current_model.deleteConstraint(up_constraint);

    std::vector<std::pair<IloRange, FloatSolution> > branches = {{down_constraint, down_solution},
                                                                 {up_constraint,   up_solution}};
    if (down_solution.size < up_solution.size) {
        std::swap(branches[0], branches[1]);
    }

    for (const auto &branch: branches) {
        current_model.addConstraint(branch.first);
        branchAndBound(current_model, branch.second);
        current_model.deleteConstraint(branch.first);
    }
    metrics.onFinishBranch();
}

void max_clique_solver::ExecutionContext::branchAndCut(CplexModel &current_model,
                                                       const FloatSolution &current_solution) {
    if (timer.is_time_over) {
        return;
    }
    current_solution.printInfo();
    metrics.onStartBranch();

    double current_solution_size = roundWithEpsilon(current_solution.size);

    if (this->optimal_solution.size > current_solution_size) {
        metrics.onDiscardedBranch();
        return;
    }

    double delta = 1e2;
    double max_upper_bound_delta = 0.0;
    double previous_upper_bound = current_solution_size;
    uint64_t iteration_period = 10;

    uint64_t iteration_count = 0;

    FloatSolution enhanced_solution = current_solution;
    while (true) {
        iteration_count++;

        if (iteration_count % iteration_period == 0) {
            if (max_upper_bound_delta > delta) {
                break;
            }
            max_upper_bound_delta = 0.0;
        }
        std::set<std::set<uint64_t>> violated_constraint = separation(current_solution);

        if (violated_constraint.empty()) {
            break;
        }

        current_model.addConstraints(violated_constraint, lower_bound, upper_bound);
        enhanced_solution = current_model.getFloatSolution();

        if (this->optimal_solution.size > roundWithEpsilon(enhanced_solution.size)) {
            metrics.onDiscardedBranch();
            return;
        }

        if (enhanced_solution.integer_variables_num == enhanced_solution.values.size()) {
            break;
        }

        max_upper_bound_delta = max(std::fabs(enhanced_solution.size - previous_upper_bound), max_upper_bound_delta);
        previous_upper_bound = enhanced_solution.size;
        current_model.reduceModel();
    }

    if (enhanced_solution.integer_variables_num == enhanced_solution.values.size()) {
        std::set<std::set<uint64_t>> violatedNonEdgeConstraints = checkSolution(enhanced_solution);
        if (violatedNonEdgeConstraints.empty()) {
            this->optimal_solution = current_solution;
            return;
        }
    }

    uint64_t branching_var = branchingFindNearestToInteger(current_solution);

    IloRange down_constraint = current_model.addEqualityConstraintToVariable(branching_var, lower_bound);
    metrics.onCplexFloatSolveStart();
    FloatSolution down_solution = current_model.getFloatSolution();
    metrics.onCplexFloatSolveFinish();
    current_model.deleteConstraint(down_constraint);

    IloRange up_constraint = current_model.addEqualityConstraintToVariable(branching_var, upper_bound);
    metrics.onCplexFloatSolveStart();
    FloatSolution up_solution = current_model.getFloatSolution();
    metrics.onCplexFloatSolveFinish();
    current_model.deleteConstraint(up_constraint);

    std::vector<std::pair<IloRange, FloatSolution> > branches = {{down_constraint, down_solution},
                                                                 {up_constraint,   up_solution}};
    if (down_solution.integer_variables_num < up_solution.integer_variables_num ||
        (down_solution.integer_variables_num == up_solution.integer_variables_num &&
         down_solution.size < up_solution.size)) {
        std::swap(branches[0], branches[1]);
        std::cout << "Order: UP, DOWN: \t("
                  << up_solution.integer_variables_num << "," << up_solution.size << "),\t("
                  << down_solution.integer_variables_num << "," << down_solution.size << ")" << std::endl;
    } else {
        std::cout << "Order: DOWN, UP \t("
                  << down_solution.integer_variables_num << "," << down_solution.size << "),\t("
                  << up_solution.integer_variables_num << "," << up_solution.size << ")" << std::endl;
    }

    for (const auto &branch: branches) {
        std::cout << "BRANCHING: \t("
                  << branch.second.integer_variables_num << "," << branch.second.size << ")" << std::endl;

        current_model.addConstraint(branch.first);
        branchAndBound(current_model, branch.second);
        current_model.deleteConstraint(branch.first);
    }
    metrics.onFinishBranch();
}


std::set<std::set<uint64_t>> max_clique_solver::ExecutionContext::separation(
        const FloatSolution &solution,
        std::size_t max_iteration) {

    auto coloring_independent_sets = graph.findWeightedIndependentSet(solution.values);

    for (const std::set<uint64_t> &S_0: coloring_independent_sets) {
        std::bitset<1024> S = localSearch(asBitset(S_0), solution.values);
        std::bitset<1024> S_best = S;

        for (std::size_t iteration = 0; iteration < max_iteration; ++iteration) {
            std::bitset<1024> S_;
            S_ = perturb(S);
            localSearch(S_, solution.values);
//            acceptance

            if (weight(S_, <#initializer#>) > weight(S, <#initializer#>)) {

            }
        }
    }
    return std::set<std::set<uint64_t>>();
}

std::set<std::set<uint64_t>> max_clique_solver::ExecutionContext::checkSolution(
        const FloatSolution &solution) {
    std::cout << "CHECK INTEGER SOLUTION:\t(" << solution.size << ", " << solution.integer_variables_num << ")"
              << std::endl;
    solution.printInfo();

    std::set<uint64_t> clique = solution.extractResult();

    std::set<std::set<uint64_t>> violatedNonEdgeConstraints;

    for (const uint64_t v1: clique) {
        for (const uint64_t v2: clique) {
            if (v1 != v2 && !(graph.confusion_matrix_bit_set_[v1][v2])) {
                violatedNonEdgeConstraints.insert(std::set<uint64_t>{v1, v2});
            }
        }
    }
    return violatedNonEdgeConstraints;
}

std::bitset<1024> max_clique_solver::ExecutionContext::localSearch(const std::bitset<1024> &current_set,
                                                                   const std::vector<double> &weights) {
    std::bitset<1024> result = current_set;
    std::bitset<1024> possible_candidates = ~result;
    std::vector<uint64_t> tightness = calculateTightness(result, possible_candidates);
    std::vector<double> weights_diff = calculateWeightsDiff(result, weights);
    std::map<uint64_t, std::bitset<1024>> candidates_1_2 = build12SwapCandidatesSet(result, possible_candidates,
                                                                                    tightness);
    for (std::pair<uint64_t, std::bitset<1024>> x_to_L_x: candidates_1_2) {
        if (x_to_L_x.second.count() < 2) {
            std::cout << "too few candidates" << std::endl;
            continue;
        }

        std::pair<uint64_t, uint64_t> swap = findFirst12Swap(x_to_L_x.first, x_to_L_x.second, weights);

        if (swap.first == UINT64_MAX) {
            std::cout << "Cannot found connected vertices" << std::endl;
            continue;
        }

        std::cout << "Increase by 1!!!" << std::endl;
        updateSetAndCandidates(result, tightness, candidates_1_2, x_to_L_x.first, swap, weights_diff,
                               std::vector<double>());
    }
    auto set_function = [&](uint64_t i, uint64_t j) {
        return (weights_diff[i] > weights_diff[j]) || (weights_diff[i] == weights_diff[j] && i < j);
    };
    std::set<uint64_t, decltype(set_function)> w1_swap_candidates(set_function);

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (result[v] || weights_diff[v] <= 0) continue;
        w1_swap_candidates.insert(v);
    }
    std::bitset<1024> deleted_neighbors;

    for (auto v: w1_swap_candidates) {
        deleted_neighbors &= graph.confusion_matrix_bit_set_[v];
        if ((deleted_neighbors & graph.confusion_matrix_bit_set_[v]).count() != 0) {
            continue;
        }
        updateSetAndCandidates(result,
                               tightness,
                               graph.confusion_matrix_bit_set_[v],
                               v,
                               weights_diff,
                               weights);
        deleted_neighbors |= graph.confusion_matrix_bit_set_[v];
    }
    return result;
}

std::bitset<1024> max_clique_solver::ExecutionContext::perturb(const std::bitset<1024> &current_set, std::size_t k) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::bitset<1024> candidates = ~current_set;
    std::uniform_int_distribution<> distrib(0, candidates.count() - 1);
    std::bitset<1024> result = current_set;

    for (std::size_t i = 0; i < k; ++i) {
        std::size_t vertex_to_insert = distrib(gen);
        std::size_t index_vertex_to_insert = 0;
        std::size_t skipped = 0;
        while (candidates[index_vertex_to_insert] && skipped != vertex_to_insert) {
            index_vertex_to_insert++;
            if (candidates[index_vertex_to_insert]) skipped++;
        }

        result = result & (~graph.confusion_matrix_bit_set_[index_vertex_to_insert]);
        result.set(index_vertex_to_insert, true);
    }
    return result;
}

std::pair<uint64_t, uint64_t> max_clique_solver::ExecutionContext::findFirst12Swap(double w_to_delete,
                                                                                   std::bitset<1024> candidates,
                                                                                   const std::vector<double> &weights) const {
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!candidates[v]) continue;
        for (std::size_t u = v + 1; v < graph.n_; ++u) {
            if (!candidates[u]) continue;
            if (!graph.confusion_matrix_bit_set_[v][u] && w_to_delete > weights[u] + weights[v]) {
                return {v, u};
            }
        }
    }
    return {UINT64_MAX, UINT64_MAX};
}

//tightness - количетво соседей не принадлежащей решению вершины которые в множестве
std::vector<uint64_t> max_clique_solver::ExecutionContext::calculateTightness(std::bitset<1024> set,
                                                                              std::bitset<1024> possible_candidates) const {
    std::vector<uint64_t> tightness(graph.n_, 0);

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (possible_candidates[v]) {
            tightness[v] = ((graph.confusion_matrix_bit_set_[v]) & set).count();
        }
    }
    return tightness;
}

std::map<uint64_t, std::bitset<1024>>
max_clique_solver::ExecutionContext::build12SwapCandidatesSet(std::bitset<1024> set,
                                                              std::bitset<1024> possible_candidates,
                                                              const std::vector<uint64_t> &tightness) const {
    std::map<uint64_t, std::bitset<1024>> candidates_per_vertex;
    std::bitset<1024> free_vertices;

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (possible_candidates[v]) {
            // посчитать сколько соседей лежат в независемом множестве
            if (tightness[v] == 0) {
                free_vertices.set(v, true);
            } else if (tightness[v] == 1) {
                std::bitset<1024> neighbor_in_independent_set = graph.confusion_matrix_bit_set_[v] & set;
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

void max_clique_solver::ExecutionContext::updateSetAndCandidates(std::bitset<1024> &current_set,
                                                                 std::vector<uint64_t> tightness,
                                                                 std::map<uint64_t, std::bitset<1024>> candidates,
                                                                 uint64_t deleted,
                                                                 std::pair<uint64_t, uint64_t> inserted,
                                                                 std::vector<double> weights_diff,
                                                                 const std::vector<double> &weights) {
    uint64_t inserted_connected_to_deleted =
            graph.confusion_matrix_bit_set_[deleted][inserted.first] +
            graph.confusion_matrix_bit_set_[deleted][inserted.second];

    tightness[deleted] = inserted_connected_to_deleted;

    current_set.set(deleted, false);
    current_set.set(inserted.first, true);
    current_set.set(inserted.second, true);

    for (const auto &neighbor: graph.adjacency_lists_[inserted.first]) {
        weights_diff[neighbor] -= weights[inserted.first];
        tightness[neighbor] += 1;
    }
    for (const auto &neighbor: graph.adjacency_lists_[inserted.second]) {
        weights_diff[neighbor] -= weights[inserted.second];
        tightness[neighbor] += 1;
    }
    for (const auto &neighbor: graph.adjacency_lists_[deleted]) {
        weights_diff[neighbor] += weights[deleted];
    }
}

void max_clique_solver::ExecutionContext::updateSetAndCandidates(std::bitset<1024> &current_set,
                                                                 std::vector<uint64_t> tightness,
                                                                 std::bitset<1024> deleted, uint64_t inserted,
                                                                 std::vector<double> weights_diff,
                                                                 const std::vector<double> &weights) {


    current_set &= ~deleted;
    current_set.set(inserted, true);

    for (const auto &neighbor: graph.adjacency_lists_[inserted]) {
        weights_diff[neighbor] -= weights[inserted];
        tightness[neighbor] += 1;
    }

    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!deleted[v]) continue;
        for (const auto &neighbor: graph.adjacency_lists_[v]) {
            weights_diff[neighbor] += weights[v];
        }
    }
}

std::vector<double>
max_clique_solver::ExecutionContext::calculateWeightsDiff(std::bitset<1024> set, const std::vector<double> &weights) {
    std::vector<double> result(weights);

    for (std::size_t i = 0; i < graph.n_; ++i) {
        std::bitset<1024> neighbor_in_independent_set = graph.confusion_matrix_bit_set_[i] & set;
        for (std::size_t j = 0; j < graph.n_; ++j) {
            if (!neighbor_in_independent_set[j]) continue;
            result[i] -= weights[j];
        }
    }
    return result;
}

double max_clique_solver::ExecutionContext::weight(std::bitset<1024> independent_set,
                                                   const std::vector<double> &weights) {
    double weight = 0.0;
    for (std::size_t v = 0; v < graph.n_; ++v) {
        if (!independent_set[v]) continue;
        weight += weights[v];
    }
    return weight;
}
