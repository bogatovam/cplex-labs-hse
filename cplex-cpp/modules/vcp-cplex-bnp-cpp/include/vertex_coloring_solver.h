#include "include/main_cplex_model.h"
#include "include/cplex_model.h"
#include "include/cql_graph.h"
#include <chrono>
#include <include/slave_cplex_model.h>
#include "csv_writer.h"

using namespace std::chrono;

namespace vertex_coloring_solver {
    class SolverMetrics {
    public:
        std::size_t current_depth = 0;
        std::size_t max_depth = 0;

        std::size_t branches_num = 0;
        std::size_t discarded_branches_num = 0;

        steady_clock::duration total_execution_time;

        void onStartBranch() {
            branches_num++;
            current_depth++;
            max_depth = max(max_depth, current_depth);
        }

        void onFinishBranch() {
            current_depth--;
        }

        void onDiscardedBranch() {
            discarded_branches_num++;
        }
    };

    class ExecutionContext {
    public:
        explicit ExecutionContext(std::size_t heuristic_size,
                                  const steady_clock::duration &time_to_execute,
                                  const Graph &graph);

        FloatSolution optimal_solution;

        SolverMetrics metrics{};

        Timer timer;

        const Graph &graph;

        static double roundUpWithEpsilon(double objective_function_value, double eps = 0.00001);

        static uint64_t branchingFindNearestToInteger(const FloatSolution &solution);

        void branchAndPrice(MainCplexModel &main_cplex_model, SlaveCplexModel &slave_cplex_model);

        void startBranchAndPrice(MainCplexModel &main_cplex_model, SlaveCplexModel &slave_cplex_model);

        std::set<WeightWithColumn> findColumnsToAddToModel(const FloatSolution &solution);

        bool generateColumnsByHeuristic(MainCplexModel &main_cplex_model,
                                        MainFloatSolution &current_solution);

        double calculateWeight(const Column &independent_set, const std::vector<double> &weights) const;

        static bool isTailingOff(double source_delta, double target_delta = 0.00001);

        bool generateColumnsByCplex(MainCplexModel &main_cplex_model, SlaveCplexModel &slave_cplex_model,
                                    MainFloatSolution &current_solution, bool b) const;
    };

    std::map<std::string, std::string> solve(const Graph &graph);

    IndependentSets solveByHeuristic(const Graph &graph);
}
