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

IloRange CplexModel::buildConstraint(const std::set<uint64_t> &constraint, const double lower_bound, const double upper_bound) {
    /*  Let's clean name & expr first  */
    names_stream.str(std::string());
    expr.clear();

    for (const auto &constraint_var: constraint) {
        names_stream << constraint_var << " + ";
        expr += x[constraint_var];
    }

    names_stream << " <= 1";
    return {env, IloNum(lower_bound), expr, IloNum(upper_bound), names_stream.str().c_str()};
}

void CplexModel::addConstraints(const std::set<std::set<uint64_t>> &constraints,
                                const double lower_bound,
                                const double upper_bound) {
    IloRangeArray constraints_to_model = IloRangeArray(env, constraints.size());

    std::size_t constraint_num = 0;
    for (const auto &constraint: constraints) {
        IloRange current_constraint = buildConstraint(constraint, lower_bound, upper_bound);

        all_constraints.push_back(current_constraint);
        constraints_to_model[constraint_num++] = buildConstraint(constraint, lower_bound, upper_bound);
    }
    cplex.getModel().add(constraints_to_model);
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
    if (!isSolved) {
        cplex.exportModel("not_solved_model.lp");
        throw std::runtime_error("It is impossible to solve CPLEX model. See 'not_solved_model.lp'");
    }
    double result = cplex.getObjValue();
    uint64_t size = x.getSize();
    std::vector<double> result_vector(size, 0.0);
    for (uint64_t i = 0; i < size; ++i) {
        result_vector[i] = cplex.getValue(x[i]);
    }
    return {result, result_vector};
}

void CplexModel::deleteConstraint(const IloRange &constraint) {
    cplex.getModel().remove(constraint);
}

void CplexModel::addConstraint(const IloRange &constraint) {
    all_constraints.push_back(constraint);
    cplex.getModel().add(constraint);
}

void CplexModel::reduceModel(std::size_t limit) {
    if (all_constraints.size() > limit) {
        all_constraints.erase(std::remove_if(
                all_constraints.begin(), all_constraints.end(),
                [&](const IloRange &constraint) {
                    double slack = cplex.getSlack(constraint);
                    if (slack > 0.0) {
                        std::cout << "Constraint " << constraint.getName() << " was deleted, slack:=" << slack
                                  << std::endl;
                        cplex.getModel().remove(constraint);
                    }
                    return slack > 0.0;
                }), all_constraints.end());
    }
}


