#include <cassert>
#include <ctime>
#include <list>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <unordered_map>

#include "../../include/domains/racetrack/RacetrackProblem.h"
#include "../../include/domains/racetrack/RTrackDetHeuristic.h"
#include "../../include/domains/sailing/SailingProblem.h"
#include "../../include/domains/DummyState.h"
#include "../../include/domains/WrapperProblem.h"

#include "../include/ppddl/mini-gpt/states.h"
#include "../include/ppddl/mini-gpt/problems.h"
#include "../include/ppddl/mini-gpt/domains.h"
#include "../include/ppddl/mini-gpt/states.h"
#include "../include/ppddl/mini-gpt/exceptions.h"
#include "../../include/ppddl/PPDDLHeuristic.h"
#include "../../include/ppddl/PPDDLProblem.h"

#include "../../include/reduced/CustomReduction.h"
#include "../../include/reduced/LeastLikelyOutcomeReduction.h"
#include "../../include/reduced/MostLikelyOutcomeReduction.h"
#include "../../include/reduced/RacetrackObviousReduction.h"
#include "../../include/reduced/ReducedHeuristicWrapper.h"
#include "../../include/reduced/ReducedModel.h"
#include "../../include/reduced/ReducedState.h"
#include "../../include/reduced/ReducedTransition.h"

#include "../../include/solvers/LAOStarSolver.h"
#include "../../include/solvers/Solver.h"
#include "../../include/solvers/VISolver.h"

#include "../../include/util/flags.h"
#include "../../include/util/general.h"

#include "../../include/Problem.h"


using namespace std;
using namespace mdplib;
using namespace mlppddl;
using namespace mlreduced;
using namespace mlsolvers;


extern int yyparse();
extern FILE* yyin;
string current_file;
int warning_level = 0;

static int verbosity = 0;
static int k = 0;
double tau = 1.2;
int l = 2;

mlcore::Problem* problem = nullptr;
mlcore::Heuristic* heuristic = nullptr;
ReducedModel* reducedModel = nullptr;
ReducedHeuristicWrapper* reducedHeuristic = nullptr;
WrapperProblem* originalProblemWrapper = nullptr;
CustomReduction* bestReductionTemplate = nullptr;
ReducedTransition* bestReduction = nullptr;
list<ReducedTransition *> reductions;


/*
 * Given the input sizes = {s1, s2, ..., sn}, this function returns all possible
 * combinations of the sets {0, 1, ... s1 - 1}, {0, 1, ..., s2 - 1}, ...,
 * {0, 1, ..., sn - 1}, across the n sets.
 *
 * For example, sizes = {2, 2, 1} computes the following list of lists
 *  {0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}.
 *
 * The result is stored in variable fullFactorialResult.
 */
void getFullFactorialIndices(vector<int> sizes,
                             list< list<int> > & fullFactorialResult) {
    if (sizes.size() == 1) {
        for (int i = 0; i < sizes[0]; i++) {
            fullFactorialResult.push_back(list<int> (1, i));
        }
        return;
    }
    getFullFactorialIndices(vector<int> (sizes.begin() + 1, sizes.end()),
                            fullFactorialResult);
    size_t prevSize = fullFactorialResult.size();
    for (int i = 0; i < sizes[0]; i++) {
        size_t j = 0;
        for (auto const oldCombination : fullFactorialResult) {
            list<int> newCombination(oldCombination);
            newCombination.push_front(i);
            fullFactorialResult.push_back(newCombination);
            if (++j == prevSize)
                break;

        }
    }
    for (size_t j = 0; j < prevSize; j++) {
        fullFactorialResult.pop_front();
    }
}


/*
 * Computes all possible (indices) combinations of l primary outcomes, across
 * actions groups of the specified sizes. The result is stored in variable
 * combinations.
 */
void getAllCombinations(vector<size_t>& groupSizes,
                             vector<vector<vector<int> > > & combinations) {
    // First we compute the possible primary outcomes combination for each
    // group
    vector < vector < vector<int> > > allCombinationsPrimaryGroups;
    vector<int> primaryCombSizes;
    for (size_t i = 0; i < groupSizes.size(); i++) {
        allCombinationsPrimaryGroups.push_back(vector <vector <int> > ());
        vector<int> currentCombination;
        for (size_t j = 0; j < l; j++)
            currentCombination.push_back(j);
        do {
            allCombinationsPrimaryGroups.back().push_back(
                vector<int> (currentCombination));
        } while (nextComb(currentCombination, groupSizes[i], l));
        primaryCombSizes.push_back(allCombinationsPrimaryGroups.back().size());
    }

    // Now we get all combinations of indices across the combinations of
    // primary outcomes
    list<list <int> > fullIndicesCombination;
    getFullFactorialIndices(primaryCombSizes, fullIndicesCombination);

    // Now we compute the full set of combinations of primary outcomes
    // matching the full indices to the corresponding set of primary outcomes
    for (auto const combinationIndices : fullIndicesCombination) {
        vector < vector<int> > newCombination;
        int groupIdx = 0;
        for (int indexPrimarySet : combinationIndices) {
            newCombination.push_back(vector<int> (
                allCombinationsPrimaryGroups.at(groupIdx++)[indexPrimarySet]));
        }
        combinations.push_back(newCombination);
    }
}


/*
 * Assigns the set of primary outcomes to use for each action group to the
 * given reduction.
 */
void assignPrimaryOutcomesToReduction(
    const vector<vector<int> > & primaryOutcomesForGroups,
    const vector<vector<mlcore::Action*> > & actionGroups,
    CustomReduction* reduction)
{
    for (size_t groupIdx = 0; groupIdx < actionGroups.size(); groupIdx++) {
        const vector<int>& primaryIndicesForGroup =
            primaryOutcomesForGroups[groupIdx];
        const vector<mlcore::Action*> & actionGroup = actionGroups[groupIdx];
        unordered_map< mlcore::Action*, vector<bool> > &
            primaryIndicatorsTempl =
                reduction->primaryIndicatorsActions();
        for (mlcore::Action* a : actionGroup) {
            for (size_t j = 0; j < primaryIndicatorsTempl[a].size(); j++) {
                primaryIndicatorsTempl[a][j] = false;
            }
            for (auto const primaryIndex : primaryIndicesForGroup) {
                primaryIndicatorsTempl[a][primaryIndex] = true;
            }
        }
    }
}


/*
 * This functions finds the best Mkl reduction, using brute force.
 */
void findBestReductionBruteForce(
    mlcore::Problem* problem, vector<vector<mlcore::Action*> > & actionGroups)
{
    // Getting all possible combination indices of l primary outcomes
    // across all action groups (each combination represents a reduction)
    vector<size_t> groupSizes;
    for (size_t i = 0; i < actionGroups.size(); i++) {
        groupSizes.push_back(
            bestReductionTemplate->
                primaryIndicatorsActions()[actionGroups[i][0]].size());

    }
    vector< vector < vector<int> > > reductions;
    getAllCombinations(groupSizes, reductions);
    // Evaluating all possible reductions
    double bestResult = mdplib::dead_end_cost + 1;
    const vector < vector<int> >* bestReduction;
    for (auto const & reduction : reductions ) {
        assert(reduction.size() == actionGroups.size());
        CustomReduction* testReduction =
            new CustomReduction(bestReductionTemplate);
        size_t groupIdx = 0;
        assignPrimaryOutcomesToReduction(reduction,
                                         actionGroups,
                                         testReduction);
        reducedModel = new ReducedModel(problem, testReduction, k);
        double result = ReducedModel::evaluateMarkovChain(reducedModel);
                                                                                dprint1(result);
        if (result < bestResult) {
                                                                                dprint2("*********", result);
            bestResult = result;
            bestReduction = &reduction;
        }
    }
                                                                                dprint1("**************************");
    assignPrimaryOutcomesToReduction(*bestReduction,
                                     actionGroups,
                                     bestReductionTemplate);
    reducedModel = new ReducedModel(problem, bestReductionTemplate, k);
    double result = ReducedModel::evaluateMarkovChain(reducedModel);
                                                                                dprint2("best result", result);
}


/*
 * Finds the best reduction on the given problem using the greedy approach
 * described in http://anytime.cs.umass.edu/shlomo/papers/PZicaps14.pdf
 *
 * This methods receives a vector< vector<mlcore::Action> > parameter that
 * allows for the same reduction to be applied to multiple actions at the
 * same time. This is useful if the actions are symmetric, for example.
 */
void findBestReductionGreedy(mlcore::Problem* problem,
                             vector<vector<mlcore::Action*> > & actionGroups)
{
    // Evaluating expected cost of full reduction
    reducedModel = new ReducedModel(problem, bestReductionTemplate, k);
    int numMCTrials = 20;
    double originalResult = ReducedModel::evaluateMarkovChain(reducedModel);
//    double previousResult = reducedModel->evaluateMonteCarlo(numMCTrials);
                                                                                dprint2("original", originalResult);
    int numOutcomes = 0;
    for (size_t i = 0; i < actionGroups.size(); i++)
        numOutcomes +=
            bestReductionTemplate->
                primaryIndicatorsActions()[actionGroups[i][0]].size();
                                                                                dprint2("num outcomes", numOutcomes);
                                                                                int cnt = 0;
    bool currentSatisfiesL = false;
    double previousResult = originalResult;
    for (int i = 0; i < numOutcomes; i++) {
        double bestResult = mdplib::dead_end_cost + 1;
        int bestGroup = -1;
        int bestOutcomeIndex = -1;
        unordered_map< mlcore::Action*, vector <bool> > &
            primaryIndicatorsActions =
                bestReductionTemplate->primaryIndicatorsActions();
        for (size_t groupIdx = 0; groupIdx < actionGroups.size(); groupIdx++) {
            vector<mlcore::Action*> & actionGroup = actionGroups[groupIdx];
            int numPrimaryInGroup = 0;
            // Testing a new reduced model for each outcome of the actions in
            // this group. We use actionGroup[0] to get the number of outcomes,
            // because all actions in the same group should have the same
            // number of outcomes
            for (size_t outcomeIdx = 0;
                 outcomeIdx < primaryIndicatorsActions[actionGroup[0]].size();
                 outcomeIdx++) {
                // The greedy method works by removing outcomes. If it is
                // already removed, there is nothing to do
                if (!primaryIndicatorsActions[actionGroup[0]][outcomeIdx])
                    continue;
                CustomReduction* testReduction =
                    new CustomReduction(bestReductionTemplate);
                unordered_map< mlcore::Action*, vector<bool> > &
                    testPrimaryIndicatorsActions =
                        testReduction->primaryIndicatorsActions();
                // Remove this outcome from the set of primary outcomes for
                // all actions in the group
                for (mlcore::Action* a : actionGroup)
                    testPrimaryIndicatorsActions[a][outcomeIdx] = false;
                reducedModel = new ReducedModel(problem, testReduction, k);
                                                                                cnt++;
                double result = ReducedModel::evaluateMarkovChain(reducedModel);
//                double result = reducedModel->evaluateMonteCarlo(numMCTrials);
                if (result < bestResult) {
                    bestGroup = groupIdx;
                    bestOutcomeIndex = outcomeIdx;
                    bestResult = result;
                }
                // Set the outcome back to true to try a new model
                for (mlcore::Action* a : actionGroup)
                    testPrimaryIndicatorsActions[a][outcomeIdx] = false;
            }
        }
                                                                                dprint2("result ", bestResult);
                                                                                dprint2("cnt ", cnt);
        // If the best reduction in this iteration increases the cost too much
        // and the current best reduction satisfies the desired number of
        // primary outcomes, then stop
        if (bestResult > tau * originalResult && currentSatisfiesL) {
            break;
        } else {
            // Update best reduction/result and keep going.
            previousResult = bestResult;
            for (mlcore::Action* a : actionGroups[bestGroup]) {
                bestReductionTemplate->
                    primaryIndicatorsActions()[a][bestOutcomeIndex] = false;
            }
        }

        // Checking if the new reduction satisfies the desired number of
        // primary outcomes
        currentSatisfiesL = true;
        for (size_t groupIdx = 0; groupIdx < actionGroups.size();
             groupIdx++) {
            vector<mlcore::Action*> & actionGroup = actionGroups[groupIdx];
            int primaryCount = 0;
            for (size_t outcomeIdx = 0;
                 outcomeIdx < primaryIndicatorsActions[actionGroup[0]].size();
                 outcomeIdx++) {
                if (primaryIndicatorsActions[actionGroup[0]][outcomeIdx])
                    primaryCount++;
            }
            if (primaryCount > l)
                currentSatisfiesL = false;
        }
    }

                                                                                unordered_map< mlcore::Action*, vector <bool> > &
                                                                                    primaryIndicatorsActions = bestReductionTemplate->primaryIndicatorsActions();
                                                                                for (size_t idx1 = 0; idx1 < actionGroups.size(); idx1++) {
                                                                                    cout << "group " << idx1 << ": ";
                                                                                    for (size_t idx2 = 0;
                                                                                         idx2 < primaryIndicatorsActions[actionGroups[idx1][0]].size();
                                                                                         idx2++) {
                                                                                        cout << primaryIndicatorsActions[actionGroups[idx1][0]][idx2] << " ";
                                                                                    }
                                                                                    cout << endl;
                                                                                }
                                                                                dprint2("greedy", previousResult);
}


ReducedTransition* chooseBestReduction(mlcore::Problem* problem)
{
    double bestValue = mdplib::dead_end_cost + 1;
    ReducedTransition* bestReduction = nullptr;
    for (auto const& reduction : reductions) {
        ReducedModel* reducedModel = new ReducedModel(problem, reduction, k);
        double valueReduction = ReducedModel::evaluateMarkovChain(reducedModel);
                                                                                dprint2((void *) reduction, valueReduction);
        if (valueReduction < bestValue) {
            bestReduction = reduction;
            bestValue = valueReduction;
        }
    }
    return bestReduction;
}


/*
 * Creates a template for racetrack reductions with 3 action groups.
 */
void createRacetrackReductionsTemplate(RacetrackProblem* rtp,
                                       CustomReduction** createdReduction)
{
    CustomReduction* reductionsTemplate = new CustomReduction(rtp);
    vector<bool> primaryIndicators;
    bool first = true;
    for (mlcore::Action* a : rtp->actions()) {
        // Setting primary outcomes for initial state
        if (first) {
            // All actions work the same for this state, use the first one
            // as template
            for (auto const & successor :
                 rtp->transition(rtp->initialState(), a)) {
                primaryIndicators.push_back(true);
            }
            reductionsTemplate->setPrimaryForState(
                rtp->initialState(), primaryIndicators);
            first = false;
        }
        primaryIndicators.clear();
        RacetrackAction* rta = static_cast<RacetrackAction*> (a);
        int numSuccessors = rtp->numSuccessorsAction(rta);
        for (int i = 0; i < numSuccessors; i++) {
            primaryIndicators.push_back((i < 2));
        }
        reductionsTemplate->setPrimaryForAction(a, primaryIndicators);
    }
    *createdReduction = reductionsTemplate;
}


/*
 * Creates a pool of reductions to choose for the racetrack problem
 */
void createReductionsPoolForRacetrack(
    RacetrackProblem* rtp, vector<vector<mlcore::Action*> > & actionGroups)
{
    // This code assumes that pslip > perr
    // most-likely-outcome determinization
    CustomReduction* reduction1 = nullptr;
    createRacetrackReductionsTemplate(
        static_cast<RacetrackProblem*> (problem), &reduction1);
    vector<vector<int> > primaryOutcomes;
    primaryOutcomes.push_back(vector<int>{1});
    primaryOutcomes.push_back(vector<int>{1});
    primaryOutcomes.push_back(vector<int>{1});
    assignPrimaryOutcomesToReduction(primaryOutcomes,
                                     actionGroups,
                                     reduction1);
    reductions.push_back(reduction1);

    // two most-likely-outcomes Mk2
    CustomReduction* reduction2 = nullptr;
    createRacetrackReductionsTemplate(
        static_cast<RacetrackProblem*> (problem), &reduction2);
    primaryOutcomes.clear();
    primaryOutcomes.push_back(vector<int>{0, 1});
    primaryOutcomes.push_back(vector<int>{0, 1});
    primaryOutcomes.push_back(vector<int>{0, 1});
    assignPrimaryOutcomesToReduction(primaryOutcomes,
                                     actionGroups,
                                     reduction2);
    reductions.push_back(reduction2);

    // least-likely-outcome determinization
    CustomReduction* reduction3 = nullptr;
    createRacetrackReductionsTemplate(
        static_cast<RacetrackProblem*> (problem), &reduction3);
    primaryOutcomes.clear();
    primaryOutcomes.push_back(vector<int>{2});
    primaryOutcomes.push_back(vector<int>{2});
    primaryOutcomes.push_back(vector<int>{2});
    assignPrimaryOutcomesToReduction(primaryOutcomes,
                                     actionGroups,
                                     reduction3);
    reductions.push_back(reduction3);

    // two least-likely-outcomes Mk2
    CustomReduction* reduction4 = nullptr;
    createRacetrackReductionsTemplate(
        static_cast<RacetrackProblem*> (problem), &reduction4);
    primaryOutcomes.clear();
    primaryOutcomes.push_back(vector<int>{2, 3});
    primaryOutcomes.push_back(vector<int>{2, 3});
    primaryOutcomes.push_back(vector<int>{2, 3});
    assignPrimaryOutcomesToReduction(primaryOutcomes,
                                     actionGroups,
                                     reduction4);
    reductions.push_back(reduction4);
}


void createSailingReductionsTemplate(SailingProblem* sp,
                                     CustomReduction* createdReduction)
{
    CustomReduction* reductionsTemplate = new CustomReduction(sp);
    vector<bool> primaryIndicators;
    bool first = true;
    for (mlcore::Action* a : sp->actions()) {
        for (int i = 0; i < 8; i++)
            primaryIndicators.push_back(true);
        reductionsTemplate->setPrimaryForAction(a, primaryIndicators);
    }
    createdReduction = reductionsTemplate;
}


void initRacetrack(string trackName, int mds, double pslip, double perror)
{
    problem = new RacetrackProblem(trackName.c_str());
    static_cast<RacetrackProblem*>(problem)->pError(perror);
    static_cast<RacetrackProblem*>(problem)->pSlip(pslip);
    static_cast<RacetrackProblem*>(problem)->mds(mds);
    heuristic = new RTrackDetHeuristic(trackName.c_str());
    static_cast<RacetrackProblem*>(problem)->useFlatTransition(true);
    problem->generateAll();
    if (verbosity > 100)
        cout << "Generated " << problem->states().size() << " states." << endl;
    createRacetrackReductionsTemplate(static_cast<RacetrackProblem*> (problem),
                                      &bestReductionTemplate);
}


void initSailing()
{
    static vector<double> costs;
    costs.push_back(1);
    costs.push_back(2);
    costs.push_back(5);
    costs.push_back(10);
    costs.push_back(mdplib::dead_end_cost + 1);

    static double windTransition[] = {
        0.20, 0.20, 0.20, 0.00, 0.00, 0.00, 0.20, 0.20,
        0.20, 0.20, 0.20, 0.20, 0.00, 0.00, 0.00, 0.20,
        0.20, 0.20, 0.20, 0.20, 0.20, 0.00, 0.00, 0.00,
        0.00, 0.20, 0.20, 0.20, 0.20, 0.20, 0.00, 0.00,
        0.00, 0.00, 0.20, 0.20, 0.20, 0.20, 0.20, 0.00,
        0.00, 0.00, 0.00, 0.20, 0.20, 0.20, 0.20, 0.20,
        0.20, 0.00, 0.00, 0.00, 0.20, 0.20, 0.20, 0.20,
        0.20, 0.20, 0.00, 0.00, 0.00, 0.20, 0.20, 0.20};

    if (!flag_is_registered_with_value("sailing-goal")) {
        cerr << "Must specify sailing-goal argument flag" << endl;
        exit(-1);
    }
    int goalSailing = atoi(flag_value("sailing-goal").c_str());

    if (!flag_is_registered_with_value("sailing-size")) {
        cerr << "Must specify sailing-size argument flag" << endl;
        exit(-1);
    }
    int sizeSailing = atoi(flag_value("sailing-size").c_str());

    if (verbosity > 100) {
        cout << "Setting up sailing domain with size " << sizeSailing <<
            " with goal " << goalSailing << endl;
    }

    problem = new SailingProblem(0, 0, 0,
                                 goalSailing, goalSailing,
                                 sizeSailing, sizeSailing,
                                 costs,
                                 windTransition,
                                 true); // using flat transition
    problem->generateAll();

    if (!flag_is_registered_with_value("heuristic") ||
            flag_value("heuristic") == "no-wind")
        heuristic =
            new SailingNoWindHeuristic(static_cast<SailingProblem*>(problem));
    createSailingReductionsTemplate(static_cast<SailingProblem*> (problem),
                                    bestReductionTemplate);
}


int main(int argc, char* args[])
{
    register_flags(argc, args);
    if (flag_is_registered("debug"))
        mdplib_debug = true;

    // Reading flags.
    assert(flag_is_registered_with_value("domain"));
    string domainName = flag_value("domain");

    if (flag_is_registered_with_value("v"))
        verbosity = stoi(flag_value("v"));

    if (flag_is_registered_with_value("k"))
        k = stoi(flag_value("k"));

    if (flag_is_registered_with_value("l"))
        l = stoi(flag_value("l"));

    int nsims = 100;
    if (flag_is_registered_with_value("n"))
        nsims = stoi(flag_value("n"));

    // Creating problem
    vector< vector<mlcore::Action*> > actionGroups;
    if (domainName == "racetrack") {
        for (int i = 0; i < 3; i++)
            actionGroups.push_back(vector<mlcore::Action*> ());
        assert(flag_is_registered_with_value("problem"));
        int mds = -1;
        double perror = 0.05;
        double pslip = 0.10;
        if (flag_is_registered_with_value("mds"))
            mds = stoi(flag_value("mds"));
        if (flag_is_registered_with_value("pslip"))
            pslip = stoi(flag_value("pslip"));
        if (flag_is_registered_with_value("perror"))
            perror = stoi(flag_value("perror"));
        string trackName = flag_value("problem");
        initRacetrack(trackName, mds, pslip, perror);
        // Actions with the same magnitude will be considered part of the
        // same group. Then the same reduction will be applied to all actions
        // in the same group
        for (mlcore::Action* a : problem->actions()) {
            RacetrackAction* rta = static_cast<RacetrackAction*> (a);
            int magnitude = abs(rta->ax()) + abs(rta->ay());
            actionGroups[magnitude].push_back(a);
        }
    } else if (domainName == "sailing") {
        initSailing();
        actionGroups.push_back(vector<mlcore::Action*> ());
        for (mlcore::Action* a : problem->actions()) {
            actionGroups[0].push_back(a);
        }
    }

    bool useFullTransition = flag_is_registered("use-full");

    ReducedTransition* bestReduction = nullptr;
    bestReduction = reductions.front();
    originalProblemWrapper = new WrapperProblem(problem);

    double totalPlanningTime = 0.0;
    mlcore::StateSet reachableStates, tipStates, subgoals;
    clock_t startTime = clock();
    if (flag_is_registered("subgoals")) {
        int depth = 4;
        if (flag_is_registered_with_value("d"))
            depth = stoi(flag_value("d"));
        getReachableStates(problem, reachableStates, tipStates, depth);
        originalProblemWrapper->overrideGoals(&tipStates);
    }
    cout << reachableStates.size() << " " << tipStates.size() << endl;
    if (flag_is_registered("use-brute-force")) {
        findBestReductionBruteForce(originalProblemWrapper, actionGroups);
        bestReduction = bestReductionTemplate;
    } else if (flag_is_registered("best-m02-racing")) {
        vector<vector<int> > primaryOutcomes;
        primaryOutcomes.push_back(vector<int>{1});
        primaryOutcomes.push_back(vector<int>{1});
        primaryOutcomes.push_back(vector<int>{0,1});
        assignPrimaryOutcomesToReduction(primaryOutcomes,
                                         actionGroups,
                                         bestReductionTemplate);
        bestReduction = bestReductionTemplate;
    } else if (flag_is_registered("best-det-racing")) {
        vector<vector<int> > primaryOutcomes;
        primaryOutcomes.push_back(vector<int>{1});
        primaryOutcomes.push_back(vector<int>{1});
        primaryOutcomes.push_back(vector<int>{1});
        assignPrimaryOutcomesToReduction(primaryOutcomes,
                                         actionGroups,
                                         bestReductionTemplate);
        bestReduction = bestReductionTemplate;
    } else if (flag_is_registered("greedy")) {
        findBestReductionGreedy(originalProblemWrapper, actionGroups);
        bestReduction = bestReductionTemplate;
    } else if (flag_is_registered("choose")) {
        reductions.push_back(
            new LeastLikelyOutcomeReduction(originalProblemWrapper, 1));
        reductions.push_back(
            new LeastLikelyOutcomeReduction(originalProblemWrapper, 2));
        reductions.push_back(
            new MostLikelyOutcomeReduction(originalProblemWrapper, 1));
        reductions.push_back(
            new MostLikelyOutcomeReduction(originalProblemWrapper, 2));
        bestReduction = chooseBestReduction(originalProblemWrapper);
    } else if (flag_is_registered("choose-racetrack")) {
        createReductionsPoolForRacetrack(
            static_cast<RacetrackProblem*> (problem), actionGroups);
        bestReduction = chooseBestReduction(originalProblemWrapper);
    }

    clock_t endTime = clock();
                                                                                dprint1("found best reduction");

    double timeReductions = double(endTime - startTime) / CLOCKS_PER_SEC;
    totalPlanningTime += timeReductions;
    cout << "time finding reductions " << timeReductions << endl;


    // Setting up the final reduced model to use
    reducedModel = new ReducedModel(problem, bestReduction, k);
    reducedHeuristic = new ReducedHeuristicWrapper(heuristic);
    reducedModel->setHeuristic(reducedHeuristic);
    static_cast<ReducedModel*>(reducedModel)->
        useFullTransition(useFullTransition);

    // We will now use the wrapper for the pro-active re-planning approach. It
    // will allow us to plan in advance for the set of successors of a
    // state-action
    originalProblemWrapper->clearOverrideGoals();

    WrapperProblem* reducedModelWrapper = new WrapperProblem(reducedModel);

    // Solving off-line using full models
    Solver* solver;
    startTime = clock();
    if (flag_is_registered("use-vi")) {
        reducedModel->generateAll();
        solver = new VISolver(reducedModel);
    } else {
        solver = new LAOStarSolver(reducedModel);
    }
    if (useFullTransition)
        solver->solve(reducedModel->initialState());
    endTime = clock();
    double timeInitialPlan = (double(endTime - startTime) / CLOCKS_PER_SEC);
    totalPlanningTime += timeInitialPlan;
    cout << "cost " << reducedModel->initialState()->cost() <<
        " time " << timeInitialPlan << endl;

    // Running a trial of the continual planning approach.
    double expectedCost = 0.0;
    double expectedTime = 0.0;
    double maxReplanningTime = 0.0;
    for (int i = 0; i < nsims; i++) {
        if (verbosity >= 10)
            cout << i << endl;
        // We don't want the simulations to re-use the computed values
        if (!useFullTransition) {
            for (mlcore::State* s : reducedModel->states())
                s->reset();
            startTime = clock();
            solver->solve(reducedModel->initialState());
            endTime = clock();
            // Initial time always counts
            expectedTime += (double(endTime - startTime) / CLOCKS_PER_SEC);
        }
        double maxReplanningTimeCurrent = 0.0;
        pair<double, double> costAndTime =
            reducedModel->trial(
                *solver, reducedModelWrapper, &maxReplanningTimeCurrent);
        expectedCost += costAndTime.first;
        maxReplanningTime = max(maxReplanningTime, maxReplanningTimeCurrent);

        // For determinization, re-planning time counts
        // (doesn't happen in parallel)
        if (flag_is_registered("best-det-racing") && k == 0)
            expectedTime += costAndTime.second;
    }
    cout << "expected cost " << expectedCost / nsims << endl;
    cout << "expected planning time " << expectedTime / nsims << endl;
    cout << "expected planning time + reductions " <<
        expectedTime / nsims + timeReductions << endl;
    cout << "max re-planning time " << maxReplanningTime << endl;

    // Releasing memory
    for (auto reduction : reductions)
        delete reduction;
    reducedModel->cleanup();
    delete reducedModel;
    originalProblemWrapper->cleanup();
    delete originalProblemWrapper;
    reducedModelWrapper->cleanup();
    delete reducedModelWrapper;
    delete problem;
    delete solver;
    return 0;
}
