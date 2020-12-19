#include "include/main_cplex_model.h"

void MainCplexModel::addColoringAsVariable(const Column &coloring) {
    std::size_t next_variable_index = model.getVariablesCount();
    for (uint64_t i = 0; i < vertex_count; ++i) {
        if (!coloring[i]) continue;
        model.addVariable(next_variable_index++);
        variable_index_to_branching_constraint.push_back(IloConstraint());
        variable_index_to_independent_set.push_back(coloring);
    }
}

void MainCplexModel::excludeColoringWithVariableIndex(std::size_t variable) {
    IloConstraint constraint = model.addLowerThanOrEqualToConstraint({variable}, 1);
    variable_index_to_branching_constraint[variable] = constraint;
}

void MainCplexModel::includeColoringWithVariableIndex(std::size_t variable) {
    IloConstraint constraint = model.addGreaterThanOrEqualToConstraint({variable}, 1);
    variable_index_to_branching_constraint[variable] = constraint;
}

void MainCplexModel::removeBranchingRestrictionsFromVariable(std::size_t variable) {
    model.deleteConstraint(variable_index_to_branching_constraint[variable]);
}

MainCplexModel::MainCplexModel(const IndependentSets &initial_colorings, std::size_t vertex_count) :
        model(initial_colorings.size(), IloNumVar::Float, IloObjective::Minimize),
        vertex_count(vertex_count) {
    variable_index_to_independent_set.reserve(initial_colorings.size() * 10);

    variable_index_to_independent_set.resize(initial_colorings.size());
    variable_index_to_branching_constraint.resize(initial_colorings.size());

    std::size_t i = 0;
    constraints = std::vector<std::set<uint64_t>>(vertex_count, std::set<uint64_t>());
    for (const auto &coloring: initial_colorings) {
        variable_index_to_independent_set[i++] = coloring;
        for (std::size_t v = 0; v < vertex_count; ++v) {
            if (!coloring[v]) continue;
            constraints[v].emplace(i);
        }
    }
    cplex_vertex_constraints = model.addGreaterThanOrEqualToConstraints(constraints, 1);
}

PrimalAndDualSolutions MainCplexModel::solveFloatProblem() {
    IloCplex solver = model.getCplexSolver();
    bool isSolved = solver.solve();
    uint64_t variables_count = model.getVariablesCount();

    double primal_obj_value = 0;
    std::vector<double> primal_variables(variables_count, 0.0);

    double dual_obj_value = 0;
    std::vector<double> dual_variables(variables_count, 0.0);

    if (!isSolved) {
        std::cout << "It is impossible to solve CPLEX model'" << std::endl;
        return {{primal_obj_value, primal_variables},
                {dual_obj_value,   dual_variables}};
    }

    primal_obj_value = solver.getObjValue();
    dual_obj_value = primal_obj_value;

    auto variables = model.getVariables();
    for (uint64_t i = 0; i < variables.size(); ++i) {
        primal_variables[i] = solver.getValue(variables[i]);
    }

    for (uint64_t i = 0; i < variables.size(); ++i) {
        dual_variables[i] = solver.getDual(cplex_vertex_constraints[i]);
    }

    return {{primal_obj_value, primal_variables},
            {dual_obj_value,   dual_variables}};
}