#include <algorithm>
#include <thread>
#include <random>
#include <include/local_search.h>
#include "include/vertex_coloring_solver.h"
#include "slave_cplex_model.h"

std::map<std::string, std::string> vertex_coloring_solver::solve(const Graph &graph) {
    std::map<std::string, std::string> log;

    steady_clock::time_point begin = steady_clock::now();
    IndependentSets heuristic = solveByHeuristic(graph);

    SlaveCplexModel slave_cplex_model(graph);
    MainCplexModel main_cplex_model(heuristic, graph.n_);

    auto heuristic_time = steady_clock::now() - begin;

    ExecutionContext bnp_context(heuristic.size(), hours(1), graph);
    bnp_context.startBranchAndPrice(main_cplex_model, slave_cplex_model);

    log["heuristic_result"] = std::to_string(heuristic.size());
    log["cplex time (sec)"] = std::to_string(duration_cast<seconds>(bnp_context.metrics.total_execution_time).count());
    log["cplex time (ms)"] = std::to_string(
            duration_cast<milliseconds>(bnp_context.metrics.total_execution_time).count());
    log["result"] = std::to_string(bnp_context.optimal_solution.size);
    log["max_depth"] = std::to_string(bnp_context.metrics.max_depth);
    log["branches_num"] = std::to_string(bnp_context.metrics.branches_num);
    log["discarded_branches_num"] = std::to_string(bnp_context.metrics.discarded_branches_num);
    log["time (sec)"] = std::to_string(
            duration_cast<seconds>(bnp_context.metrics.total_execution_time + heuristic_time).count());
    log["timeout"] = std::to_string(bnp_context.timer.is_time_over);
    return log;
}

IndependentSets vertex_coloring_solver::solveByHeuristic(const Graph &graph) {
    IndependentSets result;
    for (auto coloring_strategy: nodes_ordering_strategies) {
        IndependentSets current_independent_sets = graph.getIndependentSetByColoring(coloring_strategy);
        result.insert(current_independent_sets.begin(), current_independent_sets.end());
    }
    return result;
}

double vertex_coloring_solver::ExecutionContext::roundUpWithEpsilon(double objective_function_value,
                                                                    double eps) {
    if (isNumberInteger(objective_function_value)) {
        return objective_function_value;
    }

    double up = std::ceil(objective_function_value);
    double down = std::floor(objective_function_value);

    if (objective_function_value - eps <= down) {
        return down;
    } else {
        return up;
    }
}

uint64_t vertex_coloring_solver::ExecutionContext::branchingFindNearestToInteger(const FloatSolution &solution) {
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

vertex_coloring_solver::ExecutionContext::ExecutionContext(std::size_t heuristic_size,
                                                           const steady_clock::duration &time_to_execute,
                                                           const Graph &graph) :
        optimal_solution({static_cast<double>(heuristic_size), std::vector<double>()}),
        timer(time_to_execute),
        graph(graph) {}


void vertex_coloring_solver::ExecutionContext::startBranchAndPrice(MainCplexModel &main_cplex_model,
                                                                   SlaveCplexModel &slave_cplex_model) {
    steady_clock::time_point begin = steady_clock::now();
    branchAndPrice(main_cplex_model, slave_cplex_model);
    // TODO достать здесь значения переменных из модели и проверить решение на коррктность
    metrics.total_execution_time = steady_clock::now() - begin;
}


void vertex_coloring_solver::ExecutionContext::branchAndPrice(MainCplexModel &main_cplex_model,
                                                              SlaveCplexModel &slave_cplex_model) {
    MainFloatSolution current_solution = main_cplex_model.solveFloatProblem();
    bool is_need_to_solve_exactly = generateColumnsByHeuristic(main_cplex_model, current_solution);
    if (is_need_to_solve_exactly) {
        if (generateColumnsByCplex(main_cplex_model, slave_cplex_model, current_solution)) {
            metrics.onFinishBranch();
            metrics.onDiscardedBranch();
            return;
        }
    }
    // TODO возможно понадобится проверка на
    if (current_solution.primal.integer_variables_num == current_solution.primal.values.size()) {
        if (!is_need_to_solve_exactly) {
            bool just_to_test = generateColumnsByCplex(main_cplex_model, slave_cplex_model, current_solution);
            if (just_to_test) {
                std::cout << "WTF Why it happened!?!?!?!\n";
            }
            if (current_solution.primal.integer_variables_num == current_solution.primal.values.size()) {
                std::cout << "Found better integer solution:=(" << current_solution.primal.size << ")\n";
                this->optimal_solution = current_solution.primal;
                metrics.onFinishBranch();
                return;
            }
        } else {
            std::cout << "Found better integer solution:=(" << current_solution.primal.size << ")\n";
            this->optimal_solution = current_solution.primal;
            metrics.onFinishBranch();
            return;
        }
    }

    uint64_t branching_var = branchingFindNearestToInteger(current_solution.primal);

    main_cplex_model.includeColoringWithVariableIndex(branching_var);
    branchAndPrice(main_cplex_model, slave_cplex_model);
    main_cplex_model.removeBranchingRestrictionsFromVariable(branching_var);

    main_cplex_model.excludeColoringWithVariableIndex(branching_var);
    auto slave_forbidden_constraint = slave_cplex_model.addForbiddenSet(
            main_cplex_model.getIndependentSetAssociatedWithVariableIndex(branching_var));
    branchAndPrice(main_cplex_model, slave_cplex_model);
    slave_cplex_model.removeForbiddenSet(slave_forbidden_constraint);
    main_cplex_model.removeBranchingRestrictionsFromVariable(branching_var);

    metrics.onFinishBranch();
}


// возвращается истина, если нужно запускать полное решение
bool vertex_coloring_solver::ExecutionContext::generateColumnsByHeuristic(MainCplexModel &main_cplex_model,
                                                                          MainFloatSolution &current_solution) {
    double previous_upper_bound = roundUpWithEpsilon(current_solution.primal.size);
    while (true) {
        WeightWithColumn column_to_add = findColumnToAddToModel(current_solution.dual);
        if (column_to_add.first < 1) {
            return true;
        }
        bool isVariableExists = main_cplex_model.addColoringAsVariable(column_to_add.second);
        if (isVariableExists) {
            return true;
        }
        current_solution = main_cplex_model.solveFloatProblem();

        if (isTailingOff(std::fabs(current_solution.primal.size - previous_upper_bound))) {
            return true;
        }
        previous_upper_bound = current_solution.primal.size;
    }
}

// возвращается истина, если можно отбросить ветку
bool vertex_coloring_solver::ExecutionContext::generateColumnsByCplex(MainCplexModel &main_cplex_model,
                                                                      SlaveCplexModel &slave_cplex_model,
                                                                      MainFloatSolution &current_solution) {
    double previous_upper_bound = roundUpWithEpsilon(current_solution.primal.size);;
    while (true) {
        slave_cplex_model.updateObjectiveFunction(current_solution.dual.values);
        IntegerSolution slave_solution = slave_cplex_model.getIntegerSolution();
        double lowerBound = roundUpWithEpsilon(current_solution.dual.size / slave_solution.size);

        if (lowerBound >= optimal_solution.size) {
            return true;
        }
        if (slave_solution.size <= 1) {
            return false;
        }
        bool isVariableExists = main_cplex_model.addColoringAsVariable(slave_solution.values);
        if (isVariableExists) {
            return false;
        }
        current_solution = main_cplex_model.solveFloatProblem();

        if (isTailingOff(std::fabs(current_solution.primal.size - previous_upper_bound))) {
            return false;
        }
        previous_upper_bound = current_solution.primal.size;
    }
}

//find The Mos tWeighty Violated Constraint in dual problem
WeightWithColumn vertex_coloring_solver::ExecutionContext::findColumnToAddToModel(const FloatSolution &solution) {
    // TODO проверить, как повлияет если инициализировать одной вершиной
    auto coloring_independent_sets = graph.getWeightedIndependentSet(solution.values);
    if (coloring_independent_sets.empty()) {
        return {0, {}};
    }
    WeightToVertices best_candidate = *coloring_independent_sets.rbegin();
    // TODO check local search performance
    WeightWithColumn improved = LocalSearchLauncher::localSearch(best_candidate.second, graph,
                                                                 solution.values);
    std::pair<bool, Column> result = graph.supplementSetsToMaximumForInclusion(improved.second);
#ifdef CHECK_SOLUTION
    auto tmp = asSet(result.second, graph.n_);
    if (!graph.isVerticesIndependent(tmp)) {
        std::cout << "set is not independent" << std::endl;
        throw std::runtime_error("set is not independent");
    }
#endif
    if (result.first) {
        return {calculateWeight(result.second, solution.values), result.second};
    } else {
        return improved;
    }
}

double vertex_coloring_solver::ExecutionContext::calculateWeight(const Column &independent_set,
                                                                 const std::vector<double> &weights) {
    double result = 0.0;
    for (std::size_t i = 0; i < graph.n_; ++i) {
        if (!independent_set[i]) continue;
        result += weights[i];
    }
    return result;
}

bool vertex_coloring_solver::ExecutionContext::isTailingOff(double source_delta, double target_delta) {
    return source_delta < target_delta;
}
