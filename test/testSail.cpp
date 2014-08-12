#include <iostream>
#include <mutex>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "../include/solvers/solver.h"
#include "../include/solvers/VISolver.h"
#include "../include/solvers/LRTDPSolver.h"
#include "../include/solvers/UCTSolver.h"
#include "../include/solvers/LAOStarSolver.h"

#include "../include/util/general.h"
#include "../include/util/graph.h"

#include "../include/domains/sailing/SailingProblem.h"

using namespace mlcore;
using namespace mlsolvers;
using namespace std;

int main(int argc, char* args[])
{
    vector<double> costs;
    costs.push_back(1);
    costs.push_back(2);
    costs.push_back(5);
    costs.push_back(10);
    costs.push_back(mdplib::dead_end_cost + 1);

	double windTransition[] = {
        0.50, 0.15, 0.10, 0.00, 0.00, 0.00, 0.10, 0.15,
        0.15, 0.50, 0.15, 0.10, 0.00, 0.00, 0.00, 0.10,
        0.10, 0.15, 0.50, 0.15, 0.10, 0.00, 0.00, 0.00,
        0.00, 0.10, 0.15, 0.50, 0.15, 0.10, 0.00, 0.00,
        0.00, 0.00, 0.10, 0.15, 0.50, 0.15, 0.10, 0.00,
        0.00, 0.00, 0.00, 0.10, 0.15, 0.50, 0.15, 0.10,
        0.10, 0.00, 0.00, 0.00, 0.10, 0.15, 0.50, 0.15,
        0.15, 0.10, 0.00, 0.00, 0.00, 0.10, 0.15, 0.50};


    int size = atoi(args[1]);
    Problem* problem =
        new SailingProblem(0, 0, size -1 , size -1, size, size, costs, windTransition);

    problem->generateAll();

    cerr << problem->states().size() << " states" << endl;

    LAOStarSolver lao(problem, 0.001);
    lao.solve(problem->initialState());

    double expectedCost = 0.0;
    int numSims = 1000;
    for (int i = 0; i < numSims; i++) {
        State* tmp = problem->initialState();
        while (!problem->goal(tmp)) {
            Action* a = tmp->bestAction();
            expectedCost += problem->cost(tmp, a);
            tmp = randomSuccessor(problem, tmp, a);
        }
    }

    cerr << endl << "Estimated cost " << problem->initialState()->cost() << endl;
    cerr << "Avg. cost " << expectedCost / numSims << endl;

    delete problem;
}
