#include <algorithm>
#include <thread>
#include <random>
#include <include/local_search.h>
#include "include/vertex_coloring_solver.h"


CplexModel vertex_coloring_solver::init_cplex_model(const CqlGraph &graph,
                                                    const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &map) {
    CplexModel cplex_solver(graph.n_);

    std::set<std::set<uint64_t>> constraints;
    std::set<std::set<uint64_t>> adjacencyConstraints = buildAdjacencyConstraints(graph, strategy);
    std::set<std::set<uint64_t>> coloringConstraints = buildColoringConstraints(graph, map);

    constraints.insert(adjacencyConstraints.begin(), adjacencyConstraints.end());
    constraints.insert(coloringConstraints.begin(), coloringConstraints.end());

    cplex_solver.addConstraints(constraints, 0, 1);
    return cplex_solver;
}

std::map<std::string, std::string> vertex_coloring_solver::solve(const CqlGraph &graph) {
    std::map<std::string, std::string> log;
    std::map<NodesOrderingStrategy, std::vector<uint64_t>> coloring_by_strategy;

    steady_clock::time_point begin = steady_clock::now();

    std::bitset<1024> best_clique = getBestMaxClique(graph, coloring_by_strategy);
    CplexModel cplex_solver = init_cplex_model(graph, coloring_by_strategy, calc_strategy);

    auto heuristic_time = steady_clock::now() - begin;

    ExecutionContext bnc_context(best_clique.count(), hours(1), graph);
    bnc_context.startBranchAndPrice(cplex_solver);

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
    log["time (sec)"] = std::to_string(
            duration_cast<seconds>(bnc_context.metrics.total_execution_time + heuristic_time).count());
    log["timeout"] = std::to_string(bnc_context.timer.is_time_over);
    return log;
}

double vertex_coloring_solver::ExecutionContext::roundDownWithEpsilon(double objective_function_value,
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
                                                           const CqlGraph &graph) :
        optimal_solution({static_cast<double>(heuristic_size), std::vector<double>()}),
        timer(time_to_execute),
        graph(graph) {}

void vertex_coloring_solver::ExecutionContext::startBranchAndPrice(CplexModel &model) {
    steady_clock::time_point begin = steady_clock::now();
    branchAndPrice(model, model.getFloatSolution());
    metrics.total_execution_time = steady_clock::now() - begin;
}

void vertex_coloring_solver::ExecutionContext::branchAndPrice(CplexModel &current_model,
                                                              const FloatSolution &current_solution) {
    if (timer.is_time_over) {
        return;
    }
    metrics.onStartBranch();

    double current_solution_size = roundDownWithEpsilon(current_solution.size);

    if (this->optimal_solution.size >= current_solution_size) {
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
        iteration_count++;

        if (this->optimal_solution.size >= roundDownWithEpsilon(enhanced_solution.size)) {
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
            auto solution = current_model.getFloatSolution();
            branchAndPrice(current_model, solution);
        }
    }

//      Не особо влияет на время, поэтому лимит ограничений достаточно большой
    current_model.reduceModel();

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
        branchAndPrice(current_model, branch.second);
        current_model.deleteConstraint(branch.first);
    }
    metrics.onFinishBranch();
}

