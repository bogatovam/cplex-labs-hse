#pragma once

#include <vector>
#include "include/cplex_model.h"

class MainCplexModel {

private:

    std::size_t vertex_count;

    std::vector<std::set<uint64_t>> constraints;

    IloConstraintArray cplex_vertex_constraints;

    std::vector<IloConstraint> variable_index_to_branching_constraint;

    std::vector<Column> variable_index_to_independent_set;

    CplexModel model;

public:
    MainCplexModel(const IndependentSets &initial_colorings, std::size_t vertex_count);

    void addColoringAsVariable(const Column &coloring);

    PrimalAndDualSolutions solveFloatProblem();

    void excludeColoringWithVariableIndex(size_t variable);

    void includeColoringWithVariableIndex(size_t variable);

    void removeBranchingRestrictionsFromVariable(size_t variable);
};
