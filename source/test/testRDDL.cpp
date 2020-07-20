#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "../include/solvers/AODetHeuristic.h"
#include "../include/solvers/BoundedRTDPSolver.h"
#include "../include/solvers/DeterministicSolver.h"
#include "../include/solvers/HDPSolver.h"
#include "../include/solvers/HMinHeuristic.h"
#include "../include/solvers/HOPSolver.h"
#include "../include/solvers/LAOStarSolver.h"
#include "../include/solvers/LRTDPSolver.h"
#include "../include/solvers/FLARESSolver.h"
#include "../include/solvers/SoftFLARESSolver.h"
#include "../include/solvers/Solver.h"
#include "../include/solvers/SSiPPSolver.h"
#include "../include/solvers/UCTSolver.h"
#include "../include/solvers/VISolver.h"
#include "../include/solvers/VPIRTDPSolver.h"

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

int verbosity = 0;
Heuristic* heuristic;

clock_t startTime;
clock_t endTime;

int main(int argc, char* args[])
{
    mdplib::dead_end_cost = 1000000000000;
    register_flags(argc, args);
    
    assert(flag_is_registered_with_value("problemdir"));
    string problemDir = flag_value("problemdir");

    assert(flag_is_registered_with_value("instance"));
    string problemName = flag_value("instance");

    //build the rddl extractor and the problem
    rddlmdp::Extractor* e = new rddlmdp::Extractor(problemName, problemDir);
    Problem* problem = new RDDLProblem(e);

    //decided what we're using and go for it
    int horizon = 0, expansions = 1, trials = 1000000;
    double tol = 1.0e-3;
    if (flag_is_registered_with_value("horizon"))
        horizon = stoi(flag_value("horizon"));
    if (flag_is_registered_with_value("expansions"))
        expansions = stoi(flag_value("expansions"));
    if (flag_is_registered_with_value("trials"))
        trials = stoi(flag_value("trials"));
    if (flag_is_registered_with_value("tol"))
        tol = stof(flag_value("tol"));

    if (flag_is_registered_with_value("v"))
        verbosity = stoi(flag_value("v"));
    if (flag_is_registered_with_value("heuristic")) {
        if (flag_value("heuristic") == "hmin") {
            startTime = clock();
            bool solveAll = flag_is_registered("precompute-h");
            heuristic = new HMinHeuristic(problem, solveAll);
            endTime = clock();
	    if (solveAll) {
		if (verbosity > 100) {
		    cout << "Heuristic took " <<
			(double(endTime - startTime) / CLOCKS_PER_SEC) <<
			" seconds."  << endl;
		    cout << "Cost of initial state "
			<< problem->initialState()->cost() << endl;
		}
	    }
        } else if (flag_value("heuristic") == "aodet") {
            bool precompute = flag_is_registered("precompute-h");
            heuristic = new AODetHeuristic(problem, precompute);
        } else if (flag_value("heuristic") == "zero")
            heuristic = nullptr;
    }
    problem->setHeuristic(heuristic);

    Solver* solver;
    if (flag_is_registered_with_value("algorithm")) {
	string algorithm = flag_value("algorithm");
	if (algorithm == "lrtdp") {
	    solver = new LRTDPSolver(problem, trials, tol, -1);
	}
	else if (algorithm == "flares") {
	    bool optimal = flag_is_registered("optimal");
	    bool useProbsDepth = flag_is_registered("use-prob-depth");
	    double depth = horizon;
	    if (flag_is_registered("prob"))
		depth = stof(flag_value("prob"));
	    solver = new FLARESSolver(
				      problem, trials, tol, depth, optimal, useProbsDepth);
	}
	else if (algorithm == "laostar") {
	    solver = new LAOStarSolver(problem, tol, 1000000);
	}
	else if (algorithm == "soft-flares") {
	    double depth = horizon;
	    double alpha = 0.10;
	    bool optimal = flag_is_registered("optimal");
	    TransitionModifierFunction mod_func = kLogistic;
	    DistanceFunction dist_func = kStepDist;
	    HorizonFunction horizon_func = kFixed;
	    if (flag_is_registered_with_value("alpha"))
		alpha = stof(flag_value("alpha"));
	    // Distance functions
	    if (flag_is_registered("dist")) {
		string dist_str = flag_value("dist");
		if (dist_str == "traj") {
		    dist_func = kTrajProb;
		} else if (dist_str == "plaus") {
		    dist_func = kPlaus;
		} else if (dist_str == "depth") {
		    dist_func = kStepDist;
		} else {
		    cerr << "Error: unknown distance function." << endl;
		    exit(0);
		}
	    }
	    // Labeling functions
	    if (flag_is_registered("labelf")) {
		string labelf_str = flag_value("labelf");
		if (labelf_str == "exp") {
		    mod_func = kExponential;
		} else if (labelf_str == "step") {
		    mod_func = kStep;
		} else if (labelf_str == "linear") {
		    mod_func = kLinear;
		} else if (labelf_str == "logistic") {
		    mod_func = kLogistic;
		} else {
		    cerr << "Error: unknown labeling function." << endl;
		    exit(0);
		}
	    }
	    // Horizon functions (to allow some probability of deeper exploration)
	    double psi = 0.2;
	    if (flag_is_registered("horf")) {
		string horf_str = flag_value("horf");
		if (horf_str == "exp") {
		    horizon_func = kExponentialH;
		} else if (horf_str == "fixed") {
		    horizon_func = kFixed;
		} else if (horf_str == "bern") {
		    horizon_func = kBernoulli;
		} else {
		    cerr << "Error: unknown labeling function." << endl;
		    exit(0);
		}
	    } else if (optimal) {
		if (flag_is_registered_with_value("psi"))
		    psi = stof(flag_value("psi"));
		horizon_func = kBernoulli;
	    }
	    solver = new SoftFLARESSolver(
					  problem, trials, tol, depth, mod_func, dist_func, horizon_func,
					  alpha, false, false, optimal, psi);
	} else if (algorithm == "rtdp") {
	    solver = new SoftFLARESSolver(
					  problem, trials, tol, 0, kLinear, kStepDist, kFixed,
					  0.0, false, true);
	} else if (algorithm == "hdp") {
	    int plaus;
	    if (flag_is_registered_with_value("i"))
		solver = new HDPSolver(problem, tol, stoi(flag_value("i")));
	    else
		solver = new HDPSolver(problem, tol);
	} else if (algorithm == "vi") {
	    solver = new VISolver(problem, 1000000000, tol);
	} else if (algorithm == "ssipp") {
	    double rho = -1.0;
	    bool useTrajProb = false;
	    if (flag_is_registered_with_value("rho")) {
		rho = stof(flag_value("rho"));
		useTrajProb = true;
	    }
	    solver = new SSiPPSolver(problem, tol, horizon, SSiPPAlgo::Original);
	    SSiPPSolver* ssipp = static_cast<SSiPPSolver*> (solver);
	    ssipp->maxTrials(1);
	    ssipp->useTrajProbabilities(useTrajProb);
	    ssipp->rho(rho);
	} else if (algorithm == "labeled-ssipp") {
	    double rho = -1.0;
	    bool useTrajProb = false;
	    if (flag_is_registered_with_value("rho")) {
		rho = stof(flag_value("rho"));
		useTrajProb = true;
	    }
	    solver = new SSiPPSolver(problem, tol, horizon, SSiPPAlgo::Labeled);
	    SSiPPSolver* ssipp = static_cast<SSiPPSolver*> (solver);
	    ssipp->useTrajProbabilities(useTrajProb);
	    ssipp->rho(rho);
	} else if (algorithm == "det") {
	    solver = new DeterministicSolver(problem,
					     mlsolvers::det_most_likely,
					     heuristic);
	} else if (algorithm == "hop") {
	    solver = new HOPSolver(problem);
	    if (!flag_is_registered("heuristic")
		|| flag_value("heuristic") != "aodet") {
		cerr << "HOPSolver only works with --heuristic=aodet" << endl;
		exit(0);
	    }
	} else if (algorithm == "uct") {
	    int rollouts = 1000;
	    int cutoff = 50;
	    int delta = 5;
	    double C = 0.0;
	    bool use_qvalues_for_c = true;
	    if (flag_is_registered_with_value("rollouts"))
		rollouts = stoi(flag_value("rollouts"));
	    if (flag_is_registered_with_value("cutoff"))
		cutoff = stoi(flag_value("cutoff"));
	    if (flag_is_registered_with_value("delta"))
		delta = stoi(flag_value("delta"));
	    if (flag_is_registered("cexp")) {
		C = stod(flag_value("cexp"));
		use_qvalues_for_c = false;
	    }
	    solver = new UCTSolver(problem,
				   rollouts, cutoff, C,
				   use_qvalues_for_c, delta,
				   true);
	} else if (algorithm != "greedy") {
	    cerr << "Unknown algorithm: " << algorithm << endl;
	    exit(-1);
	}
    }
    else {
	solver = new LAOStarSolver(problem, tol, 1000000);
    }
	    
    //solve the MDP
    cout << "solving" << endl;
    startTime = clock();
    solver->solve(problem->initialState());
    endTime = clock();

    double totalTime = (double(endTime - startTime) / CLOCKS_PER_SEC);
    if (verbosity > 100) {
	cerr << "Planning time " << totalTime << endl;
    }

    int nsims = 1;
    if (flag_is_registered_with_value("nsims"))
	nsims = stoi(flag_value("nsims"));
    
    double expectedCost = 0.0;
    double expectedTime = 0.0;
    StateSet statesSeen;
    //run some sims
    cout << "running sims" << endl;
    for (int i = 0; i < nsims; i++) {
	cout << "===========================" << endl;
	cout << "TRIAL " << i << endl;
	mlcore::State* tmp = problem->initialState();
	while (!problem->goal(tmp)) {
	    statesSeen.insert(tmp);

	    if (verbosity > 999 || flag_is_registered("interactive")) {
		int count = 1;
		cout << "                                                   "
		     << "[id] action: q-value" << endl;
		cout << "                                                   "
		     << "--------------------" << endl;
		for (mlcore::Action* a : problem->actions()) {
		    if (problem->applicable(tmp, a)) {
			double qAction = std::min(mdplib::dead_end_cost, qvalue(problem, tmp, a));
			cout << "                                                   "
			     << "[" << count << "] "
			     << a << ": " << -1*qAction << endl;
		    }
		    ++count;
		}
	    }

	    Action* a;
	    if (flag_is_registered("interactive")) {
		cout << "action select" << endl;
		int input;
		cin >> input;
		int count = 1;
		if (input == 0) {
		    a = greedyAction(problem, tmp);
		}
		else {
		    for (mlcore::Action* act : problem->actions()) {
			if (problem->applicable(tmp, act) && (count == input)) {
			    a = act;
			}
			++count;
		    }
		}
	    }
	    else {
		a = greedyAction(problem, tmp);
	    }
	    int cost = problem->cost(tmp, a);
	    expectedCost += cost;
            tmp = randomSuccessor(problem, tmp, a);

	    if (verbosity > 0) {
		cout << a << endl;
		cout << "cost: " << cost << endl;
	    }
	    if (verbosity > 100) {
		cout << "updated state: " << tmp << endl;
	    }
	}
    }
    expectedCost /= nsims;
    expectedTime /= nsims;
    
    cerr << "Avg. Exec cost " << expectedCost << " ";
    cerr << "Total time " << totalTime + expectedTime << " ";
    cerr << "States seen " << statesSeen.size() << endl;
    
    delete problem;
    delete solver;
    delete ((RDDLHeuristic*) heuristic);
    delete e;
}
