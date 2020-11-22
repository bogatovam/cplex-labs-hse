#pragma once
// Magic tricks to have CPLEX behave well:
#ifndef IL_STD
#define IL_STD
#endif

#include <ilcplex/ilocplex.h>
#include <set>

#include <include/cql_graph.h>
#include "include/cplex_model.h"

class CplexModel {
private:
    // CPLEX environment. Takes care of everything, including memory management for CPLEX objects.
    IloEnv env;

    // CPLEX model. We put variables and constraints in it!
    IloModel model;

    IloNumVarArray x;

    IloExpr expr;

    std::stringstream names_stream;

    IloRange buildConstraint(const std::set<uint64_t> &constraint, uint64_t lower_bound, uint64_t upper_bound);

public:
    explicit CplexModel(std::size_t variables_num);

    void addConstraints(const std::set<std::set<uint64_t>> &constraints, uint64_t lower_bound, uint64_t upper_bound);

    void reduceModel();

    uint64_t solveInteger(const CqlGraph &graph);
};
