#include "include/cplex_model.h"
#include "include/cql_graph.h"
#include <chrono>
#include "csv_writer.h"

using namespace std::chrono;

namespace max_clique_solver {
    enum class Strategy {
        BRANCH_AND_CUT = 0,
        BRANCH_AND_BOUND = 1,
    };

    class SolverMetrics {
    public:
        std::size_t current_depth = 0;
        std::size_t max_depth = 0;

        std::size_t branches_num = 0;
        std::size_t discarded_branches_num = 0;

        steady_clock::duration total_execution_time;
        steady_clock::duration average_float_cplex_time;
        steady_clock::duration float_cplex_time;
        steady_clock::time_point float_cplex_time_start;

        void onStartBranch() {
            branches_num++;
            current_depth++;
            max_depth = max(max_depth, current_depth);
        }

        void onFinishBranch() {
            current_depth--;
        }

        void onCplexFloatSolveStart() {
            float_cplex_time_start = steady_clock::now();
        }

        void onCplexFloatSolveFinish() {
            float_cplex_time += (steady_clock::now() - float_cplex_time_start);
        }

        void onDiscardedBranch() {
            discarded_branches_num++;
        }
    };

    class ExecutionContext {
    public:
        explicit ExecutionContext(std::size_t heuristic_size,
                                  const steady_clock::duration &time_to_execute = minutes(2));

        FloatSolution optimal_solution;

        SolverMetrics metrics{};

        Timer timer;

        const double lower_bound = 0.0;

        const double upper_bound = 1.0;

        static bool isNumberInteger(double number);

        static bool isNumberCloseToInteger(double number, double eps = 0.0001);

        static double roundWithEpsilon(double objective_function_value, double eps = 0.0001);

        static bool isResultInteger(const std::vector<double> &result);

        static uint64_t branchingFindNearestToOne(const FloatSolution &solution);

        static uint64_t branchingFindNearestToZero(const FloatSolution &solution);

        static uint64_t branchingFindNearestToInteger(const FloatSolution &solution);

        static uint64_t branchingFindNearestToMiddle(const FloatSolution &solution);

    };

    class BranchAndBoundExecutionContext : public ExecutionContext {
    public:
        explicit BranchAndBoundExecutionContext(size_t heuristic_size,
                                                const steady_clock::duration &time_to_execute = minutes(2));

        void start(CplexModel &model);

        void branchAndBound(CplexModel &current_model);

        void branchAndBound(CplexModel &current_model, const FloatSolution &current_solution);
    };

    class BranchAndCutExecutionContext : public ExecutionContext {
    public:
        explicit BranchAndCutExecutionContext(size_t heuristic_size,
                                              const steady_clock::duration &time_to_execute = minutes(2));

        void start(CplexModel &model);

        void branchAndCut(CplexModel &current_model);
    };

    CplexModel
    init_cplex_model(const CqlGraph &graph, const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &map);

    std::map<std::string, std::string> solve(const CqlGraph &graph, const Strategy &calc_strategy);

    std::bitset<1024> getBestMaxClique(const CqlGraph &graph,
                                       std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring_by_strategy);

    std::set<std::set<uint64_t>> buildAdjacencyConstraints(const CqlGraph &graph);

    std::set<std::set<uint64_t>> buildColoringConstraints(const CqlGraph &graph,
                                                          const std::map<NodesOrderingStrategy, std::vector<uint64_t>> &coloring);

    void improveIndependentSet(const CqlGraph &graph,
                               const std::set<uint64_t> &independent_set,
                               std::set<std::set<uint64_t>> &result);


    void improveIndependentSetByOne(const CqlGraph &graph,
                                    const std::set<uint64_t> &independent_set,
                                    std::set<std::set<uint64_t>> &result);
}