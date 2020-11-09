#pragma once

#include <iostream>
#include <vector>

class CqlGraph {
private:
    int n, m;
    std::vector<std::vector<bool>> confusion_matrix;
    std::vector<std::vector<int>> adjacency_lists;

    void readGraph(const std::string &graphsPath, const std::string &graphName);
public:

    int getN() const;

    void setN(int n);

    int getM() const;

    void setM(int m);

    const std::vector<std::vector<bool>> &getConfusionMatrix() const;

    void setConfusionMatrix(const std::vector<std::vector<bool>> &confusionMatrix);

    const std::vector<std::vector<int>> &getAdjacencyLists() const;

    void setAdjacencyLists(const std::vector<std::vector<int>> &adjacencyLists);

    CqlGraph(const std::string &graphsPath, const std::string &graphName);
};