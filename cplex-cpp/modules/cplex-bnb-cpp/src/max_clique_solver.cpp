#include "include/max_clique_solver.h"

CplexModel max_clique_solver::init_cplex_model(const CqlGraph &graph) {
    CplexModel cplex_solver(graph.n_);


    return cplex_solver;
}

void max_clique_solver::solve(const CqlGraph &graph, const CsvWriter &csv_writer) {
    CplexModel cplex_solver = init_cplex_model(graph);
}
