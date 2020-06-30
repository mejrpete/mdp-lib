#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "../include/solvers/LAOStarSolver.h"
#include "../include/solvers/Solver.h"

#include "../include/util/flags.h"
#include "../include/util/general.h"
#include "../include/util/graph.h"

#include "../include/rddl/RDDLProblem.h"
#include "../include/rddl/RDDLState.h"
#include "../include/rddl/RDDLHeuristic.h"
#include "../include/rddl/RDDLAction.h"

using namespace mdplib;
using namespace mlcore;
using namespace mlrddl;
using namespace mlsolvers;
using namespace std;

int main(int argc, char** argv)
{
    //    register_flags(argc, args);
    //
    //    assert(flag_is_registered_with_value("track"));
    //    string trackName = flag_value("track");
    //


    //build the rddl extractor 

    string problemDir = string(argv[1]);
    string problemName = string(argv[2]);

    rddlmdp::Extractor* e = new rddlmdp::Extractor(problemName, problemDir);

    //do the work
    mdplib_debug = true;
    Problem* problem = new RDDLProblem(e);
    Heuristic* heuristic = new RDDLHeuristic((RDDLProblem*) problem);

    problem->setHeuristic(heuristic);
    
    //solve the MDP
    cout << "solving" << endl;
    clock_t startTime = clock();
    double tol = 1.0e-6;
    Solver* solver = new LAOStarSolver(problem, tol, 1000000);
    solver->solve(problem->initialState());
    clock_t endTime = clock();
    
    double actionsPerSecond = 4.0;
    double totalTime = (double(endTime - startTime) / CLOCKS_PER_SEC);
    
    int nsims = 1;
    double expectedCost = 0.0;
    double expectedTime = 0.0;
    StateSet statesSeen;
    //run some sims
    cout << "running sims" << endl;
    for (int i = 0; i < nsims; i++) {
	mlcore::State* tmp = problem->initialState();
	while (!problem->goal(tmp)) {
	    cout << tmp << endl;
	    statesSeen.insert(tmp);
	    Action* a = greedyAction(problem, tmp);
	    cout << a << endl;
	    expectedCost += problem->cost(tmp, a);
            tmp = randomSuccessor(problem, tmp, a);
	}
    }
    expectedCost /= nsims;
    expectedTime /= nsims;
    
    cerr << "Avg. Exec cost " << expectedCost << " ";
    cerr << "Total time " << totalTime + expectedTime << " ";
    double expectedCostTime = actionsPerSecond * (totalTime + expectedTime);
    cerr << "States seen " << statesSeen.size() << endl;
    
    delete problem;
    delete ((RDDLHeuristic*) heuristic);
    delete e;
}
