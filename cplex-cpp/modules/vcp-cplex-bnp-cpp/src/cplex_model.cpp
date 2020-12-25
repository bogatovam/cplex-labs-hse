#include <iosfwd>
#include <sstream>
#include <vector>
#include <include/cql_graph.h>
#include "include/cplex_model.h"


CplexModel::CplexModel(std::size_t variables_num, IloNumVar::Type variable_type, IloObjective::Sense objective_sense) {
    env = IloEnv();
    model = IloModel(env);
    expr = IloExpr(env);

    for (std::size_t i = 0; i < variables_num; ++i) {
        variables.emplace_back(env, 0, 1, variable_type, getVariableNameFromIndex(i).c_str());
    }

    IloExpr obj_expr(env);

    for (uint32_t i = 0; i < variables_num; ++i)
        obj_expr += variables[i];
    current_objective_function = IloObjective(env, obj_expr, objective_sense);
    this->objective_sense = objective_sense;
    model.add(current_objective_function);

    cplex = IloCplex(model);
    cplex.setParam(IloCplex::Param::Threads, 16);
    cplex.setParam(IloCplex::Param::Parallel, 1);
    cplex.setOut(env.getNullStream());
}

IloRange CplexModel::buildConstraint(const std::set<uint64_t> &constraint,
                                     const double lower_bound,
                                     const double upper_bound) {
    /*  Let's clean name & expr first  */
    names_stream.str(std::string());
    expr.clear();

    for (const auto &constraint_var: constraint) {
        names_stream << constraint_var << "_";
        expr += variables[constraint_var];
    }

    return {env, IloNum(lower_bound), expr, IloNum(upper_bound), names_stream.str().c_str()};
}

IloRange CplexModel::buildConstraint(const Bitset &constraint,
                                     const double lower_bound,
                                     const double upper_bound) {
    /*  Let's clean name & expr first  */
    names_stream.str(std::string());
    expr.clear();

    for (std::size_t i = 0; i < variables.size(); ++i) {
        if (!constraint[i]) continue;
        names_stream << i << "_";
        expr += variables[i];
    }
    return {env, IloNum(lower_bound), expr, IloNum(upper_bound), names_stream.str().c_str()};
}

IloRange CplexModel::buildGreaterThanOrEqualToConstraint(const std::set<uint64_t> &constraint_set,
                                                         const double greater_than_or_equal_to) {
    /*  Let's clean name & expr first  */
    names_stream.str(std::string());
    expr.clear();

    for (const auto &constraint_var: constraint_set) {
        names_stream << constraint_var << "_";
        expr += variables[constraint_var];
    }
    return IloRange(env, (IloNum) greater_than_or_equal_to, expr, IloInfinity, names_stream.str().c_str());
}

IloRangeArray CplexModel::addRangeConstraints(const std::set<std::set<uint64_t>> &constraints,
                                              const double lower_bound,
                                              const double upper_bound) {
    IloRangeArray constraints_to_model = IloRangeArray(env, constraints.size());

    std::size_t constraint_num = 0;
    for (const auto &constraint: constraints) {
        IloRange current_constraint = buildConstraint(constraint, lower_bound, upper_bound);
        all_constraints[current_constraint.getName()] = current_constraint;
        constraints_to_model[constraint_num++] = current_constraint;
    }
    cplex.getModel().add(constraints_to_model);
    return constraints_to_model;
}

IloRangeArray CplexModel::addGreaterThanOrEqualToConstraints(const std::vector<std::set<uint64_t>> &constraints,
                                                             double greater_than_or_equal_to) {
    IloRangeArray constraints_to_model = IloRangeArray(env, constraints.size());

    std::size_t constraint_num = 0;
    for (const auto &constraint: constraints) {
        IloRange current_constraint = buildGreaterThanOrEqualToConstraint(constraint, greater_than_or_equal_to);
        all_constraints[current_constraint.getName()] = current_constraint;
        constraints_to_model[constraint_num++] = current_constraint;
    }
    cplex.getModel().add(constraints_to_model);
    return constraints_to_model;
}

IloConstraint CplexModel::addEqualityConstraintToVariable(uint64_t variable, double equals_to) {
    std::string name = "x[" + std::to_string(variable) + "] = " + std::to_string(equals_to);

    expr.clear();
    expr += variables[variable];
    IloConstraint constraint = IloRange(env, (IloNum) equals_to, expr, (IloNum) equals_to, name.c_str());
    addConstraint(constraint);
    return constraint;
}

void CplexModel::deleteConstraint(const IloConstraint &constraint) {
    all_constraints.erase(constraint.getName());
    cplex.getModel().remove(constraint);
}

void CplexModel::addConstraint(const IloConstraint &constraint) {
    cplex.getModel().add(constraint);
    all_constraints[constraint.getName()] = constraint;
}

void CplexModel::reduceModel(std::size_t limit) {
    if (all_constraints.size() > limit) {
        std::vector<IloConstraint> to_delete;
        for (auto it = all_constraints.begin(); it != all_constraints.end();) {
            auto constraint = *it;
            double slack = cplex.getSlack(constraint.second);
            if (slack > 0.0) {
                to_delete.push_back(constraint.second);
                it = all_constraints.erase(it);
            } else {
                ++it;
            }
        }
        std::cout << to_delete.size() << " constraints will be deleted. Remaining constraints count:="
                  << all_constraints.size() << std::endl;

        IloConstraintArray constraints_to_model = IloConstraintArray(env, to_delete.size());
        uint64_t i = 0;
        for (const auto &c: to_delete) {
            constraints_to_model[i++] = c;
        }
        model.remove(constraints_to_model);
    }
}

void CplexModel::deleteConstraints(const IloConstraintArray &constraints) {
    for (int i = 0; i < constraints.getSize(); ++i) {
        deleteConstraint(constraints[i]);
    }
}

void CplexModel::addVariable(size_t index, double lover_bound, double upper_bound, IloNumVar::Type type) {
    variables.emplace_back(env, lover_bound, upper_bound, type, getVariableNameFromIndex(index).c_str());
    current_objective_function.setLinearCoef(variables[index], 1);
}

void CplexModel::updateObjectiveFunction(const std::vector<double> &new_coefficients) {
    cplex.getModel().remove(current_objective_function);
    IloExpr obj_expr(env);
    for (uint32_t i = 0; i < variables.size(); ++i)
        obj_expr += new_coefficients[i] * variables[i];
//        obj_expr += (equals(new_coefficients[i], 0.0) ? 1e-10 : new_coefficients[i]) * variables[i];

    current_objective_function = IloObjective(env, obj_expr, objective_sense);
    cplex.getModel().add(current_objective_function);
}

std::string CplexModel::getVariableNameFromIndex(uint64_t index) const {
    return "x_" + std::to_string(index);
}

IloCplex CplexModel::getCplexSolver(bool exact) {
    if (exact) {
        this->setCplexTimeLimitInSeconds(INT_MAX);
    } else {
        this->setCplexTimeLimitInSeconds(1);
    }
    return cplex;
}

std::size_t CplexModel::getVariablesCount() const {
    return variables.size();
}

std::size_t CplexModel::getConstraintsCount() const {
    return all_constraints.size();
}

std::map<std::string, IloConstraint> CplexModel::getConstraints() const {
    return all_constraints;
}

std::vector<IloNumVar> CplexModel::getVariables() const {
    return variables;
}

IloRange CplexModel::addGreaterThanOrEqualToConstraint(const std::set<uint64_t> &constraint,
                                                       double greater_than_or_equal_to) {
    IloRange cplex_constraint = buildGreaterThanOrEqualToConstraint(constraint, greater_than_or_equal_to);
    cplex.getModel().add(cplex_constraint);
    all_constraints[cplex_constraint.getName()] = cplex_constraint;
    return cplex_constraint;
}

IloRange CplexModel::addLowerThanOrEqualToConstraint(const std::set<uint64_t> &constraint,
                                                     double lower_than_or_equal_to) {
    IloRange cplex_constraint = buildLowerThanOrEqualToConstraint(constraint, lower_than_or_equal_to);
    cplex.getModel().add(cplex_constraint);
    all_constraints[cplex_constraint.getName()] = cplex_constraint;
    return cplex_constraint;
}

IloRange CplexModel::buildLowerThanOrEqualToConstraint(const std::set<uint64_t> &constraint,
                                                       double lower_than_or_equal_to) {
    names_stream.str(std::string());
    expr.clear();

    for (const auto &constraint_var: constraint) {
        names_stream << constraint_var << "_";
        expr += variables[constraint_var];
    }
    IloRange res = -expr >= lower_than_or_equal_to;
    res.setName(names_stream.str().c_str());
    return res;
}

void CplexModel::setCplexTimeLimitInSeconds(std::size_t seconds) {
    cplex.setParam(IloCplex::Param::TimeLimit, static_cast<IloNum >(seconds));
}

IloRange CplexModel::addRangeConstraint(const std::set<uint64_t> &constraint,
                                        double lower_bound,
                                        double upper_bound) {
    IloRange current_constraint = buildConstraint(constraint, lower_bound, upper_bound);
    all_constraints[current_constraint.getName()] = current_constraint;
    cplex.getModel().add(current_constraint);
    return current_constraint;
}

IloRange CplexModel::addRangeConstraint(const Bitset &constraint,
                                        double lower_bound,
                                        double upper_bound) {
    IloRange current_constraint = buildConstraint(constraint, lower_bound, upper_bound);
    all_constraints[current_constraint.getName()] = current_constraint;
    cplex.getModel().add(current_constraint);
    return current_constraint;
}

CplexModel::~CplexModel() {
    cplex.end();
}


