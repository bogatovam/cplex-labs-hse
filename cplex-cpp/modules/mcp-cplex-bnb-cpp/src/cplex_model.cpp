#include <iosfwd>
#include <sstream>
#include <vector>
#include <include/cql_graph.h>
#include "include/cplex_model.h"

CplexModel::CplexModel(std::size_t variables_num) {
    env = IloEnv();
    model = IloModel(env);
    x = IloNumVarArray(env, variables_num);

    for (std::size_t i = 0; i < variables_num; ++i) {
        names_stream << "x_" << i;
        x[i] = IloNumVar(env, 0, 1, IloNumVar::Float, names_stream.str().c_str());
        names_stream.str(std::string()); // Clean name
    }

    expr = IloExpr(env);
    IloExpr obj_expr(env);

    for (uint32_t i = 0; i < variables_num; ++i)
        obj_expr += x[i];
    IloObjective obj(env, obj_expr, IloObjective::Maximize);
    model.add(obj);

    cplex = IloCplex(model);
    cplex.setOut(env.getNullStream());
}

IloRange
CplexModel::buildConstraint(const std::set<uint64_t> &constraint, const double lower_bound, const double upper_bound) {
    /*  Let's clean name & expr first  */
    names_stream.str(std::string());
    expr.clear();

    for (const auto &constraint_var: constraint) {
        names_stream << constraint_var << "_";
        expr += x[constraint_var];
    }

    return {env, IloNum(lower_bound), expr, IloNum(upper_bound), names_stream.str().c_str()};
}

IloRangeArray CplexModel::addConstraints(const std::set<std::set<uint64_t>> &constraints,
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

IloRange CplexModel::addEqualityConstraintToVariable(uint64_t variable, double equals_to) {
    std::string name = "x[" + std::to_string(variable) + "] = " + std::to_string(equals_to);

    expr.clear();
    expr += x[variable];
    IloRange constraint = IloRange(env, (IloNum) equals_to, expr, (IloNum) equals_to, name.c_str());
    addConstraint(constraint);
    return constraint;
}

FloatSolution CplexModel::getFloatSolution() {
    bool isSolved = cplex.solve();
    uint64_t size = x.getSize();
    double result = 0;
    std::vector<double> result_vector(size, 0.0);
    if (!isSolved) {
        return {result, result_vector};
//        cplex.exportModel("not_solved_model.lp");
//        std::cout << "It is impossible to solve CPLEX model. See 'not_solved_model.lp'" << std::endl;
//        throw std::runtime_error("It is impossible to solve CPLEX model. See 'not_solved_model.lp'");
    }
    result = cplex.getObjValue();

    for (uint64_t i = 0; i < size; ++i) {
        result_vector[i] = cplex.getValue(x[i]);
    }
    return {result, result_vector};
}

void CplexModel::deleteConstraint(const IloRange &constraint) {
    all_constraints.erase(constraint.getName());
    cplex.getModel().remove(constraint);
}

void CplexModel::addConstraint(const IloRange &constraint) {
    cplex.getModel().add(constraint);
    all_constraints[constraint.getName()] = constraint;
}

void CplexModel::reduceModel(std::size_t limit) {
    if (all_constraints.size() > limit) {
        std::vector<IloRange> to_delete;
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

        IloRangeArray constraints_to_model = IloRangeArray(env, to_delete.size());
        uint64_t i = 0;
        for (const auto &c: to_delete) {
            constraints_to_model[i++] = c;
        }
        cplex.getModel().remove(constraints_to_model);
    }
}

void CplexModel::deleteConstraints(const IloRangeArray &constraints) {
    for (int i = 0; i < constraints.getSize(); ++i) {
        deleteConstraint(constraints[i]);
    }
}


