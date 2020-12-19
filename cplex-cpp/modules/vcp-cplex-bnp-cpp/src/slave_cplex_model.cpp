#include <include/local_search.h>
#include "slave_cplex_model.h"

std::set<std::set<uint64_t>> SlaveCplexModel::buildCliquesConstraints(const Graph &graph) const {
    Graph complement_graph = graph.buildComplementGraph();
    std::set<std::set<uint64_t>> result;
    for (auto coloring_strategy: nodes_ordering_strategies) {
        IndependentSets current_independent_sets = graph.getIndependentSetByColoring(coloring_strategy);
        for (const auto &bit_independent_set: current_independent_sets) {
            auto improved = LocalSearchLauncher::independentSetLocalSearch(bit_independent_set, complement_graph);
            std::cout << " IS ILS: " << bit_independent_set.count() << "->" << improved.first << std::endl;

            std::set<uint64_t> clique;
            for (uint64_t i = 0; i < complement_graph.n_; ++i) {
                if (!improved.second[i]) continue;
                clique.insert(i);
            }
            result.insert(clique);
        }
    }
    std::cout << "Cliques count: " << result.size() << std::endl;
    return result;
}

std::set<std::set<uint64_t>> SlaveCplexModel::buildAdjacencyConstraints(const Graph &graph) const {
    std::set<std::set<uint64_t>> result;
    for (std::size_t v = 0; v < graph.n_; ++v) {
        for (std::size_t u = v + 1; u < graph.n_; ++u) {
            if (graph.confusion_matrix_bit_set_[v][u]) {
                std::set<uint64_t> constraint;
                constraint.insert(u);
                constraint.insert(v);
                result.insert(constraint);
            }
        }
    }
    return result;
}

SlaveCplexModel::SlaveCplexModel(const Graph &graph) :
        model(graph.n_, IloNumVar::Bool, IloObjective::Maximize) {
    std::set<std::set<uint64_t>> cliques = buildCliquesConstraints(graph);
    std::set<std::set<uint64_t>> non_edges = buildAdjacencyConstraints(graph);
    std::set<std::set<uint64_t>> all_constraints(cliques.begin(), cliques.end());
    all_constraints.insert(non_edges.begin(), non_edges.end());
    model.addRangeConstraints(all_constraints);

    model.setCplexTimeLimitInSeconds(5);

    vertex_count = graph.n_;
}

void SlaveCplexModel::updateObjectiveFunction(const std::vector<double> &new_coefficients) {
    model.updateObjectiveFunction(new_coefficients);
}

IloConstraint SlaveCplexModel::addForbiddenSet(const Column &column) {
    std::set<uint64_t> set_vertices = asSet(column, vertex_count);
    IloConstraint constraint = model.addRangeConstraint(set_vertices, 0, (double) set_vertices.size() - 1);
    return constraint;
}

IntegerSolution SlaveCplexModel::getIntegerSolution() {
    IloCplex solver = model.getCplexSolver();

    bool isSolved = solver.solve();
    uint64_t variables_count = model.getVariablesCount();

    uint64_t obj_value = 0;
    std::vector<uint64_t> solver_variables(variables_count, 0);

    if (!isSolved) {
        std::cout << "It is impossible to solve CPLEX model'" << std::endl;
        return {obj_value, solver_variables};
    }

    obj_value = solver.getObjValue();
    auto variables = model.getVariables();
    for (uint64_t i = 0; i < variables.size(); ++i) {
        solver_variables[i] = solver.getValue(variables[i]);
    }

    return {obj_value, solver_variables};
}
