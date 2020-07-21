// Copyright 10.04.2018, University of Freiburg,
// Author: David Speck <speckd>.

#include "dd_heuristic_search.h"
#include "iterative_deepening_search.h"
#include "utils/stopwatch.h"
#include <algorithm>
#include <experimental/filesystem>
#include <pwd.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

using std::string;

// TODO(geisserf & speckd): Maybe we should also use some dominante action
// pruning
// or at least qValue = p(s'|s,a) * ADD_h(s') ???

DDHeuristicSearch::DDHeuristicSearch()
    : DeterministicSearchEngine("DD Heuristic Seach"),
      round_dez(2),
      timeout(0.15),
      TR_max_threshold(std::pow(10, 4.5)),
      TR_len_threshold(std::pow(10, 7.5)),
      add_state(allCPFs.size() + 1, 0),
      merge_transitions(true) {
    // Fix dummy goal variable to true
    add_state.back() = 1;
}

bool DDHeuristicSearch::setValueFromString(string& param, string& value) {
    if (param == "-p") {
        round_dez = atoi(value.c_str());
        return true;
    }
    if (param == "-timeout") {
        timeout = atof(value.c_str());
        return true;
    }
    if (param == "-merge") {
        merge_transitions = atoi(value.c_str());
        return true;
    }
    if (param == "-TRmax") {
        // string to size_t
        std::stringstream sstream(value.c_str());
        sstream >> TR_max_threshold;
        return true;
    }
    if (param == "-TRlen") {
        // string to size_t
        std::stringstream sstream(value.c_str());
        sstream >> TR_len_threshold;
        return true;
    }
    return SearchEngine::setValueFromString(param, value);
}

void DDHeuristicSearch::estimateQValue(State const& state, int actionIndex,
                                       double& qValue) {
    assert(state.stepsToGo() > 0);

    // Fix qValue estimate from IDS by subtracting layer size that DD
    // heuristic can solve
    if (state.stepsToGo() > steps_handled_by_dd) {
        qValue /= static_cast<double>(state.stepsToGo());
        qValue *= (state.stepsToGo() - steps_handled_by_dd);
    } else {
        qValue = 0;
    }

    // Get next state (apply action) and the associated reward
    double reward = 0;
    State nxt(state.stepsToGo() - 1);
    calcStateTransition(state, actionIndex, nxt, reward);

    // reward is already added if we use IDS
    if (!backup_engine || state.stepsToGo() <= steps_handled_by_dd) {
        qValue += reward;
        // std::cout << "reward adding: " << reward << std::endl;
    }

    if (steps_handled_by_dd > 0) {
        convert_state(nxt);
        qValue -= dd_heuristic.get_h(add_state, state.stepsToGo() - 1);
    }
    // std::cout << "final qValue: " << qValue << "\n" << std::endl;
}

void DDHeuristicSearch::estimateQValues(State const& state,
                                        std::vector<int> const& actionsToExpand,
                                        std::vector<double>& qValues) {
    assert(state.stepsToGo() > 0);
    if (steps_handled_by_dd == 0) {
        backup_engine->estimateQValues(state, actionsToExpand, qValues);
        return;
    }
    if (backup_engine && state.stepsToGo() > steps_handled_by_dd) {
        backup_engine->estimateQValues(state, actionsToExpand, qValues);
    }
    for (unsigned int index = 0; index < qValues.size(); ++index) {
        if (actionsToExpand[index] == index) {
            estimateQValue(state, index, qValues[index]);
        }
    }
}

void DDHeuristicSearch::convert_state(State const& state) {
    for (size_t index = 0; index < State::numberOfDeterministicStateFluents;
         ++index) {
        add_state[index] = state.deterministicStateFluent(index);
        // converted_state.push_back(static_cast<int>(
        //    std::round(state.deterministicStateFluent(index))));
    }
    int const& num_det = State::numberOfDeterministicStateFluents;
    for (size_t index = 0; index < State::numberOfProbabilisticStateFluents;
         ++index) {
        add_state[index + num_det] = state.probabilisticStateFluent(index);
        // converted_state.push_back(static_cast<int>(
        //    std::round(state.probabilisticStateFluent(index))));
    }
}

void DDHeuristicSearch::learn(long time) {
    long learn_time = timeout * time;
    std::cout << "DD_Heuristic: learning [" << learn_time / 1000 << "s ("
              << timeout << "%)] ";

    struct passwd* pw = getpwuid(getuid());
    std::string homedir(pw->pw_dir);
    std::string task_file = homedir + "/" + taskName + ".json";
    std::cout << "with " << task_file << "..." << std::endl;
    std::cout << "Horizon: " << horizon << std::endl;
    std::cout << "Round to dezimal: " << round_dez << std::endl;

    // Build task with timeout
    try {
        task.parse_task(task_file, round_dez, learn_time, TR_max_threshold,
                        TR_len_threshold, merge_transitions);
        std::experimental::filesystem::remove(task_file);
    } catch (const std::exception& e) {
        std::cout << "Not able to build symbolic structures." << std::endl;
        std::cout << "=> Changing to IDS." << std::endl;
        // This one is critical => we kill the Cudd mgr for memory
        // => after this line of code we should never look up anything of cudd
        std::cout << "\nRAM USED: " << SystemUtils::getRAMUsedByThis()
                  << std::endl;
        task.reset(true);
        std::cout << "sleeping...";
        sleep(10);
        std::cout << "done!" << std::endl;
        std::cout << "\nRAM USED: " << SystemUtils::getRAMUsedByThis()
                  << std::endl;
        backup_engine = std::make_unique<IDS>();
        backup_engine->setStrictTerminationTimeout(0.25);
        backup_engine->setTimeout(0.05);
        backup_engine->learn(time);
        name += " => IDS";
        search_d = horizon;
        steps_handled_by_dd = 0;
        return;
    }

    // Only if we were able to build up the task
    try {
        dd_heuristic.compute_dd_heuristic(task, horizon);
    } catch (const std::exception& e) {
        std::cout << "Not able to build all layers." << std::endl;
    }
    // Check num_layers
    std::cout << "Completed layers: " << dd_heuristic.num_layers() << std::endl;
    // Unset timelimit of Cudd
    Cudd_UnsetTimeLimit(task.get_cudd_mgr()->getManager());
    if (dd_heuristic.num_layers() - 1 < horizon) {
        std::cout << "=> Use Combination" << std::endl;
        backup_engine = std::make_unique<IDS>();
        backup_engine->learn(time);
        task.reset(false);

        // Max depth is the minimium of horizon - layers done and the learned
        // depth
        steps_handled_by_dd = static_cast<int>(dd_heuristic.num_layers()) - 1;
        search_d = std::min(backup_engine->getMaxSearchDepth(),
                            horizon - steps_handled_by_dd);
        std::cout << "IDS DEPTH = min(" << horizon - steps_handled_by_dd << ", "
                  << backup_engine->getMaxSearchDepth() << ") = " << search_d
                  << std::endl;
        this->name += "[Steps: " + std::to_string(steps_handled_by_dd) +
                      "] + IDS [Steps: " + std::to_string(search_d) + "]";
        backup_engine->setMaxSearchDepth(search_d);
        return;
    }
    task.reset(false);
    steps_handled_by_dd = horizon;
    this->name += "[Steps: " + std::to_string(steps_handled_by_dd) + "]";
    std::cout << "... finished" << std::endl;
}

void DDHeuristicSearch::disableCaching() {
    // Disable caching for backup engine if it exists
    if (backup_engine) {
        backup_engine->disableCaching();
    }
    SearchEngine::disableCaching();
}

void DDHeuristicSearch::printStats(std::ostream& out,
                                   bool const& printRoundStats,
                                   std::string indent) const {
    SearchEngine::printStats(out, printRoundStats, indent);
    if (backup_engine) {
        backup_engine->printStats(out, printRoundStats, indent);
    }
}
