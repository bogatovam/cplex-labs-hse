#pragma once

#include <iostream>
#include <vector>

class CqlGraph {
private:
    int n, m;
    std::vector<std::vector<bool>> confusion_matrix;
    std::vector<std::vector<int>> adjacency_lists;

public:
    static CqlGraph readGraph(const std::string &graphsPath, const std::string &graphName);

    CqlGraph(std::vector<std::vector<bool>> confusionMatrix, std::vector<std::vector<int>> adjacencyLists,
             int n, int m);
};