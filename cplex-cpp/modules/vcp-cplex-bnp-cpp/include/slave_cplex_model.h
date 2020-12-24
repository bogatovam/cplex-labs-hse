#pragma once

#include <include/cplex_model.h>

class SlaveCplexModel {
private:
    CplexModel model;

    std::size_t vertex_count;

    static std::set<std::set<uint64_t>> buildCliquesConstraints(const Graph &graph) ;

    static std::set<std::set<uint64_t>> buildAdjacencyConstraints(const Graph &graph) ;

public:
    explicit SlaveCplexModel(const Graph &graph);

    void updateObjectiveFunction(const std::vector<double> &new_coefficients);

    IloConstraint addForbiddenSet(const Bitset &set_vertices);

    IntegerSolution getIntegerSolution(bool exact);

    void removeForbiddenSet(const IloConstraint &constraint);
};
