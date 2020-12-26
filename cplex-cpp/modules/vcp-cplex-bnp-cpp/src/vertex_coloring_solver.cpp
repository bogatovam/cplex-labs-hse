#include <algorithm>
#include <thread>
#include <random>
#include <stack>
#include <include/local_search.h>
#include "include/vertex_coloring_solver.h"

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
        for (auto is: current_independent_sets) {
            if (std::find(result.begin(), result.end(), is) == result.end()) {
                result.emplace_back(is);
            }
        }
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

uint64_t vertex_coloring_solver::ExecutionContext::branchingFindNearestToInteger(
        const FloatSolution &solution) {
    std::pair<uint64_t, double> nearest_to_int = {UINT64_MAX, 1.0};
    for (uint64_t i = 0; i < solution.values.size(); ++i) {
        if (isNumberInteger(solution.values[i])) {
            continue;
        }
        double diff = min(1.0 - solution.values[i], solution.values[i]);
        if (lessThan(diff, nearest_to_int.second)) {
            nearest_to_int.first = i;
            nearest_to_int.second = diff;
        }
    }
    return nearest_to_int.first;
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
    metrics.total_execution_time = steady_clock::now() - begin;

    std::set<uint64_t> result = optimal_solution.extractResult();
    IndependentSets res = main_cplex_model.getVariablesByIds(result);

    std::vector<uint64_t> colored_vertices(graph.n_, 0);
    uint64_t color = 0;
    for (auto set: res) {
        for (std::size_t j = 0; j < graph.n_; ++j) {
            if (!set[j]) continue;
            colored_vertices[j] = color;
        }
        color++;
    }

    std::cout << "Found result coloring" << std::endl;
    for (auto h: res) {
        for (std::size_t i = 0; i < graph.n_; ++i) {
            if (!h[i]) continue;
            std::cout << i << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

#ifdef CHECK_SOLUTION
    if (!graph.isColoringCorrect(colored_vertices)) {
        std::cout << "this color_to_vertex_bitset is not correct" << std::endl;
        throw std::runtime_error("this color_to_vertex_bitset is not correct");
    }
#endif
}

void vertex_coloring_solver::ExecutionContext::branchAndPrice(MainCplexModel &main_cplex_model,
                                                              SlaveCplexModel &slave_cplex_model) {
    if (timer.is_time_over) {
        return;
    }

    metrics.onStartBranch();
    MainFloatSolution current_solution = main_cplex_model.solveFloatProblem();

    if (current_solution.primal.size == 0 &&
        current_solution.primal.integer_variables_num == current_solution.primal.values.size()) {
        std::cout << "\nDISCARD BRANCH: there is no solution" << std::endl;
        metrics.onFinishBranch();
        metrics.onDiscardedBranch();
        return;
    }

    bool is_need_to_solve_exactly = generateColumnsByHeuristic(main_cplex_model, current_solution);
    if (is_need_to_solve_exactly) {
        if (generateColumnsByCplex(main_cplex_model, slave_cplex_model, current_solution, false)) {
            metrics.onFinishBranch();
            metrics.onDiscardedBranch();
            return;
        }
    }
    if (current_solution.primal.integer_variables_num == current_solution.primal.values.size()) {
        if (generateColumnsByCplex(main_cplex_model, slave_cplex_model, current_solution, true)) {
            metrics.onFinishBranch();
            metrics.onDiscardedBranch();
            return;
        }
        if (current_solution.primal.integer_variables_num == current_solution.primal.values.size() &&
            greaterThan(this->optimal_solution.size, current_solution.primal.size)) {
            std::cout << "\nBETTER INTEGER SOLUTION:=(" << current_solution.primal.size << ")" << std::endl;
            this->optimal_solution = current_solution.primal;
        }
        metrics.onFinishBranch();
        return;
    }
    uint64_t branching_var = branchingFindNearestToInteger(current_solution.primal);

    auto include_constraint = main_cplex_model.includeColoringWithVariableIndex(branching_var);
    branchAndPrice(main_cplex_model, slave_cplex_model);
    main_cplex_model.removeConstraint(include_constraint);

    auto exclude_constraint = main_cplex_model.excludeColoringWithVariableIndex(branching_var);
    auto slave_forbidden_constraint = slave_cplex_model.addForbiddenSet(
            main_cplex_model.getIndependentSetAssociatedWithVariableIndex(branching_var));
    branchAndPrice(main_cplex_model, slave_cplex_model);
    slave_cplex_model.removeForbiddenSet(slave_forbidden_constraint);
    main_cplex_model.removeConstraint(exclude_constraint);
    metrics.onFinishBranch();
}


// возвращается истина, если нужно запускать полное решение
bool vertex_coloring_solver::ExecutionContext::generateColumnsByHeuristic(MainCplexModel &main_cplex_model,
                                                                          MainFloatSolution &current_solution) {
    double previous_upper_bound = roundUpWithEpsilon(current_solution.primal.size);

    uint64_t iterations = 0;
    uint64_t total_variables = 0;
    while (true) {
        iterations++;
        auto columns_to_add = findColumnsToAddToModel(current_solution.dual);
        if (columns_to_add.empty()) {
            return true;
        }
        bool isAllVariablesExists = true;
        for (auto column_to_add: columns_to_add) {
            isAllVariablesExists &= main_cplex_model.addColoringAsVariable(column_to_add.column);
        }
        if (isAllVariablesExists) {
            return true;
        }
        total_variables += columns_to_add.size();
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
                                                                      MainFloatSolution &current_solution,
                                                                      bool exact) const {
    double previous_upper_bound = roundUpWithEpsilon(current_solution.primal.size);;
    while (true) {
        slave_cplex_model.updateObjectiveFunction(current_solution.dual.values);
        IntegerSolution slave_solution = slave_cplex_model.getIntegerSolution(exact);
        slave_solution.values = graph.supplementSetsToMaximumForInclusion(slave_solution.values);
        double lower_bound = roundUpWithEpsilon(current_solution.primal.size / slave_solution.upper_bound);
        if (!lessThan(lower_bound, optimal_solution.size)) {
            return true;
        }
        if (!greaterThan(slave_solution.upper_bound, 1.0)) {
            return false;
        }
        // Для отслеживания добавления множеств совпадающих с запрещеннымидостаточно проверки на наличие
        // в существующих переменных. Случай добавления множества включаещего запрещенное невозможен, так как
        // все кандидаты на включение дополняются до максимальных по включению
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

std::set<WeightWithColumn> vertex_coloring_solver::ExecutionContext::findColumnsToAddToModel(
        const FloatSolution &solution) {
    std::set<WeightWithColumn> result_columns;
    auto coloring_independent_sets = graph.getWeightedIndependentSet(solution.values);

    for (const auto &current_coloring: coloring_independent_sets) {
        WeightWithColumn improved = LocalSearchLauncher::localSearch(current_coloring.column, graph,
                                                                     solution.values);
        Column final_wis = graph.supplementSetsToMaximumForInclusion(improved.column);
#ifdef CHECK_SOLUTION
        if (!graph.isVerticesIndependent(final_wis)) {
            std::cout << "set is not independent" << std::endl;
            throw std::runtime_error("set is not independent");
        }
#endif
        double final_weight = calculateWeight(final_wis, solution.values);
        if (greaterThan(final_weight, 1.0)) {
            result_columns.emplace(final_weight, final_wis);
        }
    }
    return result_columns;
}

double vertex_coloring_solver::ExecutionContext::calculateWeight(const Column &independent_set,
                                                                 const std::vector<double> &weights) const {
    double result = 0.0;
    for (std::size_t i = 0; i < graph.n_; ++i) {
        result += weights[i] * independent_set[i];
    }
    return result;
}

bool vertex_coloring_solver::ExecutionContext::isTailingOff(double source_delta, double target_delta) {
    return source_delta < target_delta;
}
