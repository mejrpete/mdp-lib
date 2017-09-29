#include <climits>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include "../include/domains/ctp/CTPOptimisticHeuristic.h"
#include "../include/domains/ctp/CTPProblem.h"
#include "../include/domains/ctp/CTPState.h"
#include "../include/domains/gridworld/GridWorldProblem.h"
#include "../include/domains/gridworld/GWManhattanHeuristic.h"
#include "../include/domains/racetrack/RacetrackProblem.h"
#include "../include/domains/racetrack/RTrackDetHeuristic.h"
#include "../include/domains/sailing/SailingNoWindHeuristic.h"
#include "../include/domains/sailing/SailingProblem.h"
#include "../include/solvers/HMinHeuristic.h"
#include "../include/solvers/LAOStarSolver.h"
#include "../include/solvers/Solver.h"
#include "../include/solvers/thts/THTSSolver.h"
#include "../include/solvers/thts/THTSWrapperHeuristic.h"
#include "../include/util/flags.h"
#include "../include/util/general.h"
#include "../include/util/graph.h"

using namespace mdplib;
using namespace mlcore;
using namespace mlsolvers;
using namespace std;

Problem* problem = nullptr;
Heuristic* heuristic = nullptr;
THTSSolver* solver = nullptr;;

int verbosity = 0;
bool useOnline = false;

unordered_map<string, THTSBackup> string_backup_function_map({
    {"mc", MONTE_CARLO},
    {"max-mc", MAX_MONTE_CARLO},
    {"pb", PARTIAL_BELLMAN}
});

void setupRacetrack()
{
    string trackName = flag_value("track");
    if (verbosity > 100)
        cout << "Setting up racetrack " << trackName << endl;
    int mds = -1;
    if (flag_is_registered_with_value("mds"))
        mds = stoi(flag_value("mds"));
    problem = new RacetrackProblem(trackName.c_str());
    RacetrackProblem* rtp = static_cast<RacetrackProblem*>(problem);
    rtp->pError(0.20);
    rtp->pSlip(0.10);
    rtp->mds(mds);
    rtp->keepSingleCopy(true);
    if (!flag_is_registered_with_value("heuristic") ||
            flag_value("heuristic") == "domain")
        heuristic = new RTrackDetHeuristic(trackName.c_str());
}


void setupGridWorld()
{
    string grid = flag_value("grid");
    if (verbosity > 100)
        cout << "Setting up grid world " << grid << endl;
    problem = new GridWorldProblem(grid.c_str(), 1.0, 50.0, true);
    if (!flag_is_registered_with_value("heuristic") ||
            flag_value("heuristic") == "domain")
        heuristic = new GWManhattanHeuristic((GridWorldProblem*) problem);
}


void setupSailingDomain()
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

    int sizeSailing = atoi(flag_value("sailing-size").c_str());
    int goalSailing = atoi(flag_value("sailing-goal").c_str());

    if (verbosity > 100)
        cout << "Setting up sailing domain with size " << sizeSailing <<
            " with goal " << goalSailing << endl;

    problem =
        new SailingProblem(0, 0, 0,
                           goalSailing, goalSailing,
                           sizeSailing, sizeSailing,
                           costs,
                           windTransition);

    if (!flag_is_registered_with_value("heuristic") ||
            flag_value("heuristic") == "domain")
        heuristic =
            new SailingNoWindHeuristic(static_cast<SailingProblem*>(problem));
}


void setupCTP()
{
    if (verbosity > 100) {
        cout << "Setting up Canadian Traveler Problem " <<
            flag_value("ctp") << endl;
    }
    problem = new CTPProblem(flag_value("ctp").c_str());
    if (!flag_is_registered_with_value("heuristic") ||
            flag_value("heuristic") == "domain")
        heuristic =
            new CTPOptimisticHeuristic(static_cast<CTPProblem*> (problem));
}


void setupProblem()
{
    if (verbosity > 100)
        cout << "Setting up problem" << endl;
    if (flag_is_registered_with_value("track")) {
        setupRacetrack();
    } else if (flag_is_registered_with_value("grid")) {
        setupGridWorld();
    } else if (flag_is_registered_with_value("sailing-size")) {
        setupSailingDomain();
    } else if (flag_is_registered_with_value("ctp")) {
        setupCTP();
    } else {
        cerr << "Invalid problem." << endl;
        exit(-1);
    }
}


bool mustReplan(State* s, int plausTrial) {
    return true;
}


void initSolver()
{
    assert(flag_is_registered_with_value("action-sel"));

    int horizon = 5, trials = 1000, virtual_rollouts = 5;
    if (flag_is_registered_with_value("horizon"))
        horizon = stoi(flag_value("horizon"));
    if (flag_is_registered_with_value("trials"))
        trials = stoi(flag_value("trials"));

    THTSHeuristic* thts_heuristic =
        new THTSWrapperHeuristic(problem, heuristic);
    solver = new THTSSolver(
        problem, thts_heuristic, trials, horizon, horizon, virtual_rollouts);

    if (flag_is_registered_with_value("backup")) {
        assert(string_backup_function_map.count(flag_value("backup")));
        solver->backupFunction(
            string_backup_function_map[flag_value("backup")]);
    }
}


void updateStatistics(double cost, int n, double& mean, double& M2)
{
    double delta = cost - mean;
    mean += delta / n;
    M2 += delta * (cost - mean);
}


int main(int argc, char* args[])
{
    register_flags(argc, args);
    if (flag_is_registered_with_value("v"))
        verbosity = stoi(flag_value("v"));
    long seed = time(nullptr);
    if (verbosity >= 1000)
        cout << "Seed: " << seed << endl;
    if (flag_is_registered("debug"))
        mdplib_debug = true;
    setupProblem();
    if (!flag_is_registered("dont-generate"))
        problem->generateAll();
    if (flag_is_registered_with_value("heuristic")) {
        if (flag_value("heuristic") == "hmin") {
            clock_t startTime = clock();
            bool solveAll = flag_is_registered("hmin-solve-all");
            heuristic = new HMinHeuristic(problem, solveAll);
            clock_t endTime = clock();
            if (verbosity > 100) {
                cout << "Heuristic took " <<
                    (double(endTime - startTime) / CLOCKS_PER_SEC) <<
                    " seconds."  << endl;
            }
        } else if (flag_value("heuristic") == "zero")
            heuristic = nullptr;
    }
    problem->setHeuristic(heuristic);

    if (verbosity > 100)
        cout << problem->states().size() << " states" << endl;

    initSolver();

    int nsims = 100;
    if (flag_is_registered_with_value("n"))
        nsims = stoi(flag_value("n"));

    // Running simulations to evaluate the solver's performance.
    double expectedCost = 0.0;
    double variance = 0.0;
    double expectedTime = 0.0;
    StateSet statesSeen;

    int cnt = 0;
    int numDecisions = 0;
                                                                                LAOStarSolver lao_solver(problem, 1.0e-6);
    for (int i = 0; i < nsims; i++) {
        if (verbosity >= 100)
            cout << " ********* Simulation Starts ********* " << endl;
        clock_t startTime, endTime;
        if (i == 0 && !flag_is_registered("no-initial-plan")) {
            for (State* s : problem->states())
                s->reset();
            startTime = clock();
            solver->solve(problem->initialState());
            endTime = clock();
            expectedTime += (double(endTime - startTime) / CLOCKS_PER_SEC);
            numDecisions++;
        }
        if (verbosity >= 10) {
            cout << "Starting simulation " << i << endl;
        }
        State* tmp = problem->initialState();
        if (verbosity >= 100) {
            cout << "Estimated cost " <<
                problem->initialState()->cost() << endl << tmp << " ";
        }
        double costTrial = 0.0;
        int plausTrial = 0;
        while (!problem->goal(tmp)) {
            statesSeen.insert(tmp);
            startTime = clock();
            Action* a = solver->solve(tmp);
                                                                                //Action* lao_action = lao_solver.solve(tmp);
                                                                                //if (lao_action != a) {
                                                                                //    cout << "Wrong action: " << a << " should be " << lao_action << endl;
                                                                                //}
            endTime = clock();
            expectedTime += (double(endTime - startTime) / CLOCKS_PER_SEC);
            numDecisions++;

            if (verbosity >= 1000) {
                cout << "State/Action: " << tmp << " " << a << " " << endl;
            }
            costTrial += problem->cost(tmp, a);
            if (costTrial >= mdplib::dead_end_cost) {
                break;
            }
            double prob = 0.0;
            State* aux = randomSuccessor(problem, tmp, a, &prob);
            tmp = aux;
        }
        if (!flag_is_registered("keep-search-tree"))
            solver->delete_tree();
        if (verbosity >= 1000) {
            cout << "Final State: " << tmp << endl;
        }
        if (flag_is_registered("ctp")) {
            CTPState* ctps = static_cast<CTPState*>(tmp);
            if (!ctps->badWeather()) {
                cnt++;
                updateStatistics(costTrial, cnt, expectedCost, variance);
            }
        } else {
            cnt++;
            updateStatistics(costTrial, cnt, expectedCost, variance);
            if (verbosity >= 1)
                cout << costTrial << endl;
        }
        if (verbosity >= 100)
            cout << endl;
    }

    if (verbosity >= 1) {
        cout << "Estimated cost " << problem->initialState()->cost() << " ";
        cout << "Avg. Exec cost " << expectedCost << " ";
        cout << "Std. Dev. " << sqrt(variance / (cnt - 1)) << " ";
        cout << "Total time " << expectedTime / cnt << " " << endl;
        cout << "States seen " << statesSeen.size() << endl;
        cout << "Avg. time per decision " <<
            expectedTime / numDecisions << endl;
    } else {
        cout << problem->initialState()->cost() << " ";
        cout << expectedCost << " " << sqrt(variance / (cnt - 1)) << " " <<
            expectedTime / cnt << " " << expectedTime / numDecisions << endl;
    }
    delete problem;
    delete heuristic;
    delete solver;
}
