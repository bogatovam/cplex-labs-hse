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
                                  const steady_clock::duration &time_to_execute,
                                  const CqlGraph &graph);

        FloatSolution optimal_solution;

        SolverMetrics metrics{};

        Timer timer;

        const CqlGraph &graph;

        const double lower_bound = 0.0;

        const double upper_bound = 1.0;

        static double roundWithEpsilon(double objective_function_value, double eps = 0.00001);

        static bool isResultInteger(const std::vector<double> &result);

        static uint64_t branchingFindNearestToOne(const FloatSolution &solution);

        static uint64_t branchingFindNearestToZero(const FloatSolution &solution);

        static uint64_t branchingFindNearestToInteger(const FloatSolution &solution);

        static uint64_t branchingFindNearestToMiddle(const FloatSolution &solution);

        void start(CplexModel &model);

        void branchAndBound(CplexModel &current_model);

        void branchAndBound(CplexModel &current_model, const FloatSolution &current_solution);

        void branchAndCut(CplexModel &current_model, const FloatSolution &current_solution);

        std::set<std::set<uint64_t>> separation(const FloatSolution &solution, std::size_t max_iteration = 10);

        std::set<std::set<uint64_t>> checkSolution(const FloatSolution &solution);

        std::bitset<1024> localSearch(const std::bitset<1024> &current_set,
                                      const std::vector<double> &weights);

        std::bitset<1024> perturb(const std::bitset<1024> &current_set, std::size_t k = 1);

        std::vector<uint64_t> calculateTightness(std::bitset<1024> set, std::bitset<1024> possible_candidates) const;

        std::map<uint64_t, std::bitset<1024>>
        build12SwapCandidatesSet(std::bitset<1024> set, std::bitset<1024> possible_candidates,
                                 const std::vector<uint64_t> &tightness) const;

        std::pair<uint64_t, uint64_t> findFirst12Swap(double w_to_delete, std::bitset<1024> candidates,
                                                      const std::vector<double> &weights) const;

        void updateSetAndCandidates(std::bitset<1024> &current_set,
                                    std::vector<uint64_t> tightness,
                                    std::map<uint64_t, std::bitset<1024>> candidates,
                                    uint64_t deleted,
                                    std::pair<uint64_t, uint64_t> inserted,
                                    std::vector<double> weights_diff,
                                    const std::vector<double>& weights);

        std::vector<double> calculateWeightsDiff(std::bitset<1024> set, const std::vector<double> &weights);

        void updateSetAndCandidates(std::bitset<1024> &current_set,
                                    std::vector<uint64_t> tightness,
                                    std::bitset<1024> deleted, uint64_t inserted,
                                    std::vector<double> weights_diff,
                                    const std::vector<double> &weights);

        double weight(std::bitset<1024> independent_set, const std::vector<double> &weights);
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