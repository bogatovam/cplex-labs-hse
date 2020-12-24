#pragma once
#ifndef IL_STD
#define IL_STD
#endif

#include <ilcplex/ilocplex.h>
#include <set>

#include <include/cql_graph.h>
#include "shared.h"


class CplexModel {
private:

    IloEnv env;

    IloModel model;

    IloExpr expr;

    std::stringstream names_stream;

    IloCplex cplex;

    std::map<std::string, IloConstraint> all_constraints;

    std::vector<IloNumVar> variables;

    IloObjective current_objective_function;
    IloObjective::Sense objective_sense;

    IloRange buildConstraint(const std::set<uint64_t> &constraint,
                             double lower_bound,
                             double upper_bound);

    IloRange buildGreaterThanOrEqualToConstraint(const std::set<uint64_t> &constraint_set,
                                                 const double greater_than_or_equal_to);

public:
    CplexModel(size_t variables_num, IloNumVar::Type variable_type, IloObjective::Sense objective_sense);

    IloRangeArray addRangeConstraints(const std::set<std::set<uint64_t>> &constraints,
                                      const double lower_bound = 0,
                                      const double upper_bound = 1);

    IloRange addRangeConstraint(const std::set<uint64_t> &constraint,
                                double lower_bound = 0,
                                double upper_bound = 1);

    IloRange addRangeConstraint(const Bitset &constraint,
                                double lower_bound = 0,
                                double upper_bound = 1);

    void reduceModel(std::size_t limit = 5000);

    IloConstraint addEqualityConstraintToVariable(uint64_t variable, double equals_to);

    void deleteConstraint(const IloConstraint &constrain);

    void addConstraint(const IloConstraint &constraint);

    void deleteConstraints(const IloConstraintArray &constraints);

    IloRangeArray addGreaterThanOrEqualToConstraints(const std::vector<std::set<uint64_t>> &constraints,
                                                     double greater_than_or_equal_to);

    IloRange addGreaterThanOrEqualToConstraint(const std::set<uint64_t> &constraint,
                                               double greater_than_or_equal_to);

    IloRange addLowerThanOrEqualToConstraint(const std::set<uint64_t> &constraint,
                                             double lower_than_or_equal_to);

    std::string getVariableNameFromIndex(uint64_t index) const;

    void addVariable(size_t index,
                     double lover_bound = 0,
                     double upper_bound = 1,
                     IloNumVar::Type type = IloNumVar::Float);

    IloCplex getCplexSolver(bool exact = true);

    std::size_t getVariablesCount() const;

    std::size_t getConstraintsCount() const;

    std::vector<IloNumVar> getVariables() const;

    std::map<std::string, IloConstraint> getConstraints() const;

    IloRange buildLowerThanOrEqualToConstraint(const std::set<uint64_t> &constraint,
                                               double lower_than_or_equal_to);

    void setCplexTimeLimitInSeconds(std::size_t seconds = 5);

    void updateObjectiveFunction(const std::vector<double> &new_coefficients);

    IloRange buildConstraint(const Bitset &constraint, double lower_bound, const double upper_bound);
};
