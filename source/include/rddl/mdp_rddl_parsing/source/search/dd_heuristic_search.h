// Copyright 10.04.2018, University of Freiburg,
// Author: David Speck <speckd>.

#ifndef DD_HEURISTIC_SEARCH_H_
#define DD_HEURISTIC_SEARCH_H_

#include "dd_heuristic/dd_heuristic.h"
#include "dd_heuristic/det_task.h"
#include "search_engine.h"
#include <vector>

class IDS;

class DDHeuristicSearch : public DeterministicSearchEngine {
public:
    DDHeuristicSearch();

    // Set parameters from command line
    bool setValueFromString(std::string& param, std::string& value) override;

    // Start the search engine to estimate the Q-value of a single action
    void estimateQValue(State const& state, int actionIndex,
                        double& qValue) override;

    // Start the search engine to estimate the Q-values of all applicable
    // actions
    void estimateQValues(State const& state,
                         std::vector<int> const& actionsToExpand,
                         std::vector<double>& qValues) override;

    void learn(long time) override;

    // This is called when caching is disabled because memory becomes sparse.
    void disableCaching() override;

    void printStats(std::ostream& out, bool const& printRoundStats,
                    std::string indent) const override;

private:
    // Parameter
    DDHeuristic dd_heuristic;
    // Threshold to round probabilities
    int round_dez;
    double timeout; // % of total time used for dd_heuristic
    size_t TR_max_threshold;
    size_t TR_len_threshold;
    DetTask task;

    std::vector<int> add_state;
    std::unique_ptr<IDS> backup_engine;
    int search_d;
    int steps_handled_by_dd;
    bool merge_transitions;

    void convert_state(State const& state);
};

#endif // DD_HEURISTIC_SEARCH_H_
