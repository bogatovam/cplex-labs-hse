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
}

IloRange CplexModel::buildConstraint(const std::set<uint64_t> &constraint, uint64_t lower_bound, uint64_t upper_bound) {
    /*  Let's clean name & expr first  */
    names_stream.str(std::string());
    expr.clear();

    names_stream << "constraint_";
    for (const auto &constraint_var: constraint) {
        names_stream << constraint_var << "_";
        expr += x[constraint_var];
    }

    return {env, IloNum(lower_bound), expr, IloNum(upper_bound), names_stream.str().c_str()};
}

void CplexModel::addConstraints(const std::set<std::set<uint64_t>> &constraints,
                                uint64_t lower_bound,
                                uint64_t upper_bound) {
    IloRangeArray constraints_to_model = IloRangeArray(env, constraints.size());

    std::size_t constraint_num = 0;
    for (const auto &constraint: constraints) {
        constraints_to_model[constraint_num++] = buildConstraint(constraint, lower_bound, upper_bound);
    }
    model.add(constraints_to_model);
}

void CplexModel::reduceModel() {

}

uint64_t CplexModel::solveInteger(const CqlGraph &graph) {
    IloCplex cplex(model);
    cplex.exportModel("model.lp");

    cplex.setOut(env.getNullStream());
    bool solved = cplex.solve();

    std::vector<uint64_t> res;
    std::vector<double> vars;

    auto n = graph.n_;
    auto obj_val = cplex.getObjValue();
    if (solved) {
        for (int i = 0; i < n; ++i) {
            auto val = cplex.getValue(x[i]);
            vars.push_back(val);
            if (val)
                res.emplace_back(i);
        }
    }

    std::cout << obj_val << std::endl;
    for (auto v: res) {
        std::cout << v << ",\t";
    }
    std::cout << std::endl;
    return 0;
}
