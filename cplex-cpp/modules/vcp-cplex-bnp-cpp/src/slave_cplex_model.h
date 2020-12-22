#pragma once

#include <include/cplex_model.h>

class SlaveCplexModel {
private:
    CplexModel model;

    std::size_t vertex_count;

    std::set<std::set<uint64_t>> buildCliquesConstraints(const Graph &graph) const;

    std::set<std::set<uint64_t>> buildAdjacencyConstraints(const Graph &graph) const;

public:
    explicit SlaveCplexModel(const Graph &graph);

    void updateObjectiveFunction(const std::vector<double> &new_coefficients);

    IloConstraint addForbiddenSet(const Bitset &set_vertices);

    IntegerSolution getIntegerSolution();

    void turnOffTimeout();

    void setTimeout(size_t seconds);

    void removeForbiddenSet(const IloConstraint &constraint);
};
