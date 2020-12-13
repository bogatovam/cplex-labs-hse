#include "include/cplex_model.h"
#include "include/cql_graph.h"
#include <chrono>
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
                                  const CqlGraph &graph);

        FloatSolution optimal_solution;

        SolverMetrics metrics{};

        Timer timer;

        const CqlGraph &graph;

        const double lower_bound = 0.0;

        const double upper_bound = 1.0;

        static double roundDownWithEpsilon(double objective_function_value, double eps = 0.00001);

        static uint64_t branchingFindNearestToInteger(const FloatSolution &solution);

        void branchAndPrice(CplexModel &current_model, const FloatSolution &current_solution);

        void startBranchAndPrice(CplexModel &model);
    };

    CplexModel init_cplex_model(const CqlGraph &graph,
                                const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &map);

    std::map<std::string, std::string> solve(const CqlGraph &graph);
}