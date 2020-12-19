#pragma once

#include <vector>
#include "include/cplex_model.h"

class MainCplexModel {

private:

    std::size_t vertex_count;

    std::vector<std::set<uint64_t>> constraints;

    IloConstraintArray cplex_vertex_constraints;

    std::vector<IloConstraint> variable_index_to_branching_constraint;

    std::vector<std::set<uint64_t>> variable_index_to_independent_set;

    CplexModel model;

    bool addColoringAsVariable(const std::set<uint64_t> &coloring_as_set_of_vertices);

public:
    MainCplexModel(const IndependentSets &initial_colorings, std::size_t vertex_count);

    // возвращают флаг, существует ли переменная в модели
    bool addColoringAsVariable(const Column &coloring);

    bool addColoringAsVariable(const std::vector<uint64_t> &coloring);

    MainFloatSolution solveFloatProblem();

    void excludeColoringWithVariableIndex(size_t variable);

    void includeColoringWithVariableIndex(size_t variable);

    void removeBranchingRestrictionsFromVariable(size_t variable);

    void switchToNewConstraint(size_t variable_index);

    std::set<uint64_t> getIndependentSetAssociatedWithVariableIndex(size_t variable_index);
};
