#include <algorithm>
#include <thread>
#include <random>
#include <include/local_search.h>
#include "include/max_clique_solver.h"

#define CHECK_SOLUTION

std::set<std::set<uint64_t>>
max_clique_solver::buildAdjacencyConstraints(const CqlGraph &graph, const Strategy strategy) {
    std::set<std::set<uint64_t>> result;
    if (strategy == Strategy::BRANCH_AND_BOUND) {
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
        std::cout << "CHECK" << std::endl;
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
                                               const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &map,
                                               const Strategy strategy) {
    CplexModel cplex_solver(graph.n_);

    std::set<std::set<uint64_t>> constraints;
    std::set<std::set<uint64_t>> adjacencyConstraints = buildAdjacencyConstraints(graph, strategy);
    std::set<std::set<uint64_t>> coloringConstraints = buildColoringConstraints(graph, map);

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
    CplexModel cplex_solver = init_cplex_model(graph, coloring_by_strategy, calc_strategy);

    auto heuristic_time = steady_clock::now() - begin;

    ExecutionContext bnc_context(best_clique.count(), hours(1), graph);
    bnc_context.startBranchAndCut(cplex_solver);

    log["heuristic_time (sec)"] = std::to_string(duration_cast<seconds>(heuristic_time).count());
    log["heuristic_time (ms)"] = std::to_string(duration_cast<milliseconds>(heuristic_time).count());
    log["heuristic_result"] = std::to_string(best_clique.count());
    log["cplex time (sec)"] = std::to_string(duration_cast<seconds>(bnc_context.metrics.total_execution_time).count());
    log["cplex time (ms)"] = std::to_string(
            duration_cast<milliseconds>(bnc_context.metrics.total_execution_time).count());
    log["result"] = std::to_string(bnc_context.optimal_solution.size);
    log["max_depth"] = std::to_string(bnc_context.metrics.max_depth);
    log["branches_num"] = std::to_string(bnc_context.metrics.branches_num);
    log["discarded_branches_num"] = std::to_string(bnc_context.metrics.discarded_branches_num);
    log["average_float_cplex_time"] =
            std::to_string(duration_cast<milliseconds>(bnc_context.metrics.average_float_cplex_time).count());
    log["float_cplex_time"] =
            std::to_string(duration_cast<milliseconds>(bnc_context.metrics.float_cplex_time).count());
    log["time (sec)"] = std::to_string(
            duration_cast<seconds>(bnc_context.metrics.total_execution_time + heuristic_time).count());
    log["timeout"] = std::to_string(bnc_context.timer.is_time_over);
    log["average_cutting_time"] =
            std::to_string(duration_cast<seconds>(bnc_context.metrics.average_cutting_time).count());
    log["average_cutting_iteration_time"] =
            std::to_string(duration_cast<seconds>(bnc_context.metrics.average_cutting_iteration_time).count());
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

void max_clique_solver::ExecutionContext::startBranchAndBound(CplexModel &model) {
    steady_clock::time_point begin = steady_clock::now();
    branchAndBound(model, model.getFloatSolution());
    metrics.total_execution_time = steady_clock::now() - begin;
    metrics.average_float_cplex_time = metrics.float_cplex_time / metrics.branches_num;
}

void max_clique_solver::ExecutionContext::startBranchAndCut(CplexModel &model) {
    steady_clock::time_point begin = steady_clock::now();
    branchAndCut(model, model.getFloatSolution());
    metrics.total_execution_time = steady_clock::now() - begin;
    metrics.average_float_cplex_time = metrics.float_cplex_time / metrics.branches_num;
    metrics.average_cutting_time = metrics.cutting_time / metrics.branches_num;
    metrics.average_cutting_iteration_time = metrics.average_cutting_iteration_time / metrics.branches_num;
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
//    if (timer.is_time_over) {
//        return;
//    }
    metrics.onStartBranch();

    double current_solution_size = roundWithEpsilon(current_solution.size);

    if (this->optimal_solution.size > current_solution_size) {
        metrics.onDiscardedBranch();
        return;
    }

    double delta = 0.01;
    double max_upper_bound_delta = 0;
    double previous_upper_bound = current_solution_size;

    uint64_t iteration_count = 0;

    FloatSolution enhanced_solution = current_solution;
    metrics.onCuttingStart();
    while (true) {
        metrics.onCuttingIterationStart();
//        std::cout << "\t Iteration # " << iteration_count << " , " << std::endl;
        iteration_count++;

        if (this->optimal_solution.size >= roundWithEpsilon(enhanced_solution.size)) {
            metrics.onDiscardedBranch();
            metrics.onCuttingIterationEnd();
            metrics.onCuttingEnd(iteration_count);
            metrics.onFinishBranch();
            return;
        }

        std::set<std::set<uint64_t>> violated_constraint = separation(current_solution);

        if (violated_constraint.empty()) {
            break;
        }

        current_model.addConstraints(violated_constraint, lower_bound, upper_bound);
        enhanced_solution = current_model.getFloatSolution();

        max_upper_bound_delta = std::fabs(enhanced_solution.size - previous_upper_bound);
        previous_upper_bound = enhanced_solution.size;
        current_model.reduceModel();

        if (max_upper_bound_delta < delta) {
            break;
        }
        metrics.onCuttingIterationEnd();
    }
    metrics.onCuttingEnd(iteration_count);

    if (enhanced_solution.integer_variables_num == enhanced_solution.values.size()) {
        std::set<std::set<uint64_t>> violatedNonEdgeConstraints = checkSolution(enhanced_solution);
        if (violatedNonEdgeConstraints.empty()) {
            std::cout << "Found better integer solution " << "(" << enhanced_solution.size << " , "
                      << enhanced_solution.integer_variables_num << ")\n";
            this->optimal_solution = enhanced_solution;
            metrics.onFinishBranch();
            return;
        } else {
            auto constraints = current_model.addConstraints(violatedNonEdgeConstraints, lower_bound, upper_bound);
            branchAndCut(current_model, current_model.getFloatSolution());
            current_model.deleteConstraints(constraints);
        }
    }

    uint64_t branching_var = branchingFindNearestToInteger(enhanced_solution);

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
    }

    for (const auto &branch: branches) {
        current_model.addConstraint(branch.first);
        branchAndCut(current_model, branch.second);
        current_model.deleteConstraint(branch.first);
    }
    metrics.onFinishBranch();
}


std::set<std::set<uint64_t>> max_clique_solver::ExecutionContext::separation(const FloatSolution &solution) {
    std::set<std::set<uint64_t>> result;
    auto coloring_independent_sets = graph.findWeightedIndependentSet(solution.values);

    for (const std::pair<double, std::set<uint64_t >> &init: coloring_independent_sets) {
        std::pair<double, std::bitset<1024>> improved = LocalSearchLauncher::localSearch(asBitset(init.second), graph,
                                                                                         solution.values);
        if (improved.first > init.first) {

//            std::cout << "Independent set was improved. Weight:= " << init.first << "->" << improved.first <<
//                      ");\t" << "; From:=(";
//            for (const auto &v: init.second) {
//                std::cout << v << ", ";
//            }
//            std::cout << "); \t To:=(";
//            for (const auto &v: tmp) {
//                std::cout << v << ", ";
//            }
//            std::cout << ");" << std::endl;

//#ifdef CHECK_SOLUTION
//            auto tmp = asSet(improved.second, graph.n_);
//            if (!graph.isVerticesIndependent(tmp)) {
//                std::cout << "set is not independent" << std::endl;
//                throw std::runtime_error("set is not independent");
//            }
//#endif
            result.insert(asSet(improved.second, graph.n_));
        } else {
            result.insert(init.second);
        }
    }
    return result;
}

std::set<std::set<uint64_t>> max_clique_solver::ExecutionContext::checkSolution(
        const FloatSolution &solution) {
    std::cout << "CHECK INTEGER SOLUTION:\t(" << solution.size << ", " << solution.integer_variables_num << ")"
              << std::endl;

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

