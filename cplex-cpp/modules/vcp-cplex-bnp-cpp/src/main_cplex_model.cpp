#include "include/main_cplex_model.h"

bool MainCplexModel::addColoringAsVariable(const Column &coloring) {
    for (const auto &existing_variable: variable_index_to_independent_set) {
        if (existing_variable == coloring) {
            std::cout << "\nVariable already exists:\t";
            for (std::size_t j = 0; j < vertex_count; ++j) {
                if (!coloring[j]) continue;
                std::cout << j << ",\t";
            }
            std::cout << std::endl;
            printModelStatistic();
            return true;
        }
    }

    std::size_t next_variable_index = model.getVariablesCount();
    model.addVariable(next_variable_index);
    variable_index_to_branching_constraint.resize(variable_index_to_branching_constraint.size() + 1);
    variable_index_to_independent_set.push_back(coloring);

    for (std::size_t i = 0; i < vertex_count; ++i) {
        if (!coloring[i]) continue;
        constraints[i].emplace(next_variable_index);
        switchToNewConstraint(i);
    }

    printModelStatistic();
    return false;
}

void MainCplexModel::switchToNewConstraint(std::size_t variable_index) {
    model.deleteConstraint(cplex_vertex_constraints[variable_index]);
    cplex_vertex_constraints[variable_index] =
            model.addGreaterThanOrEqualToConstraint(constraints[variable_index], 1);
}


void MainCplexModel::excludeColoringWithVariableIndex(std::size_t variable) {
    IloConstraint constraint = model.addLowerThanOrEqualToConstraint({variable}, 1);
    variable_index_to_branching_constraint[variable] = constraint;
    std::cout << "\nBRANCHING by variable:=\t" << variable << "\texcluding coloring...";
}

void MainCplexModel::includeColoringWithVariableIndex(std::size_t variable) {
    IloConstraint constraint = model.addGreaterThanOrEqualToConstraint({variable}, 1);
    variable_index_to_branching_constraint[variable] = constraint;
    std::cout << "\nBRANCHING by variable:=\t" << variable << "\tincluding coloring...";
}

void MainCplexModel::removeBranchingRestrictionsFromVariable(std::size_t variable) {
    std::cout << "\nBRANCHING:  reset variable:=\t" << variable << "\t";
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
        variable_index_to_independent_set[i] = coloring;
        for (std::size_t v = 0; v < vertex_count; ++v) {
            if (!coloring[v]) continue;
            constraints[v].emplace(i);
        }
        i++;
    }
    cplex_vertex_constraints = model.addGreaterThanOrEqualToConstraints(constraints, 1);
    std::cout << "\nMaster model created...\n";
}

MainFloatSolution MainCplexModel::solveFloatProblem() {
    printModelStatistic();
    IloCplex solver = model.getCplexSolver();
    bool isSolved = solver.solve();
    uint64_t variables_count = model.getVariablesCount();

    double primal_obj_value = 0;
    std::vector<double> primal_variables(variables_count, 0.0);

    double dual_obj_value = 0;
    std::vector<double> dual_variables(cplex_vertex_constraints.getSize(), 0.0);

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
    IloNumArray dual_vars(solver.getEnv(), cplex_vertex_constraints.getSize());
    solver.getDuals(dual_vars, cplex_vertex_constraints);
    for (int i = 0; i < cplex_vertex_constraints.getSize(); ++i) {
        dual_variables[i] =  dual_vars[i];
    }

    return {{primal_obj_value, primal_variables},
            {dual_obj_value,   dual_variables}};
}

Bitset MainCplexModel::getIndependentSetAssociatedWithVariableIndex(size_t variable_index) {
    return variable_index_to_independent_set[variable_index];
}

void MainCplexModel::printModelStatistic() const {
    std::cout << "\n\n------------Model statistics------------\n";
    std::cout << "\n---------------Constraints--------------\n";
    for (uint64_t i = 0; i < constraints.size(); ++i) {
        std::cout << i << ":\t";
        for (auto t2: constraints[i]) {
            std::cout << "x[" << t2 << "] + \t";
        }
        std::cout << ">=1\n";
    }
    std::cout << std::endl;
    std::cout << "\n----------Branching Constraints---------\n";
    for (uint64_t i = 0; i < variable_index_to_branching_constraint.size(); ++i) {
        if (variable_index_to_branching_constraint[i].getImpl() != nullptr) {
            std::cout << i << ":\t";
            std::cout << variable_index_to_branching_constraint[i].getName() << " \t";
            std::cout << "\n";
        }
    }
    std::cout << std::endl;
    std::cout << "\n----------------Variables---------------\n";
    for (std::size_t i = 0; i < variable_index_to_independent_set.size(); ++i) {
        std::cout << "x[" << i << "]=\t(";
        for (std::size_t j = 0; j < vertex_count; ++j) {
            if (!variable_index_to_independent_set[i][j]) continue;
            std::cout << j << ", ";
        }
        std::cout << ")\n";
    }
    std::cout << std::endl;
}
