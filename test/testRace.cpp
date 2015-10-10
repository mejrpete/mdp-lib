#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "../include/solvers/solver.h"
#include "../include/solvers/VISolver.h"
#include "../include/solvers/LRTDPSolver.h"
#include "../include/solvers/UCTSolver.h"
#include "../include/solvers/LAOStarSolver.h"
#include "../include/solvers/DeterministicSolver.h"

#include "../include/util/general.h"
#include "../include/util/graph.h"

#include "../include/domains/racetrack/RacetrackProblem.h"
#include "../include/domains/racetrack/RacetrackState.h"
#include "../include/domains/racetrack/RacetrackAction.h"
#include "../include/domains/racetrack/RTrackDetHeuristic.h"
#include "../include/domains/racetrack/RTrackLowResHeuristic.h"

using namespace mlcore;
using namespace mlsolvers;
using namespace std;

// Usage: ./testrace problem algorithm nsims
int main(int argc, char* args[])
{
    Problem* problem = new RacetrackProblem(args[1]);
    ((RacetrackProblem*) problem)->setPError(0.10);
    ((RacetrackProblem*) problem)->setPSlip(0.20);
    ((RacetrackProblem*) problem)->setMDS(-1);
    problem->generateAll();

    Heuristic* heuristic = new RTrackDetHeuristic(args[1]);

    problem->setHeuristic(heuristic);

    cerr << problem->states().size() << " states" << endl;

    DeterministicSolver det(problem, mlsolvers::det_most_likely, heuristic);
    clock_t startTime = clock();
    double tol = 1.0e-6;
    if (strcmp(args[2], "wlao") == 0) {
        LAOStarSolver wlao(problem, tol, 1000000, atof(args[4]));
        wlao.solve(problem->initialState());
    } else if (strcmp(args[2], "lao") == 0) {
        LAOStarSolver lao(problem, tol, 1000000);
        lao.solve(problem->initialState());
    } else if (strcmp(args[2], "lrtdp") == 0) {
        LRTDPSolver lrtdp(problem, 1000000000, tol);
        lrtdp.solve(problem->initialState());
    } else if (strcmp(args[2], "vi") == 0) {
        VISolver vi(problem, 1000000000, tol);
        vi.solve();
    } else if (strcmp(args[2], "det") != 0) {
        cerr << "Unknown algorithm: " << args[2] << endl;
        return -1;
    }
    clock_t endTime = clock();

    cerr << "Estimated cost " << problem->initialState()->cost() << endl;
    double costTime = (double(endTime - startTime) / CLOCKS_PER_SEC) * 4.0;
    cerr << "Planning Time: " <<  costTime / 4.0 << endl;

    if (strcmp(args[2], "vi") == 0) {
        for (State* s : problem->states()) {
            if (s->cost() < heuristic->cost(s)) {
                cerr << "Error: " << s << " " << s->cost() << " " << heuristic->cost(s) << endl;
            }
        }
    }

    int nsims = atoi(args[3]);
    int verbosity = 1000;
    double expectedCost = 0.0;
    for (int i = 0; i < nsims; i++) {
        State* tmp = problem->initialState();
        if (verbosity > 100) {
            cerr << " ********* Simulation Starts ********* " << endl;
            cerr << tmp << " ";
        }
        while (!problem->goal(tmp)) {
            Action* a;
            if (strcmp(args[2], "det") == 0) {
                startTime = clock();
                a = det.solve(tmp);
                endTime = clock();
                costTime += (double(endTime - startTime) / CLOCKS_PER_SEC) * 4.0;
            } else {
                a = tmp->bestAction();
            }

            expectedCost += problem->cost(tmp, a);
            tmp = randomSuccessor(problem, tmp, a);
            if (verbosity > 100) {
                cerr << a << " " << endl;
                cerr << tmp << " ";
            }
        }
        if (verbosity > 100)
            cerr << endl;
    }

    cerr << "Avg. Exec cost " << expectedCost / nsims << endl;
    cerr << "Avg.Total cost " << expectedCost / nsims + costTime << endl;

    delete problem;
    delete ((RTrackDetHeuristic*) heuristic);
}
