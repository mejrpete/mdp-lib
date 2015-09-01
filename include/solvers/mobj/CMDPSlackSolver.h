#ifndef MDPLIB_CMDPSLACKSOLVER_H
#define MDPLIB_CMDPSLACKSOLVER_H

#include <vector>

#include "../../mobj/mobj_problem.h"
#include "../solver.h"
#include "CMDPSolver.h"

namespace mlsolvers
{

/**
 * This class transforms the LMDP with global slack into a series of CMDPs to
 * obtain the optimal policy within the alotted slack.
 */
class CMDPSlackSolver : public Solver
{
private:

    mlmobj::MOProblem* problem_;

    CMDPSolver* internalSolver_;

    RandomPolicy* policy_;

    /* Error tolerance */
    std::vector<double> slack_;

public:

    /**
     * Creates a CMDPSlackSolver for the given problem.
     *
     * @param problem The problem to be solved.
     * @param slack The vector of global slacks.
     */
    CMDPSlackSolver(mlmobj::MOProblem* problem, std::vector<double> slack)
        : problem_(problem), slack_(slack), internalSolver_(nullptr) { }

    ~CMDPSlackSolver()
    {
        delete internalSolver_;
    }

    /**
     * Solves the associated problem using the CMDPSlackSolver algorithm.
     *
     * @param s0 The state to start the search at.
     */
    mlcore::Action* solve(mlcore::State* s0);

    /**
     * Returns the optimal policy for the problem.
     */
    RandomPolicy* policy() { return policy_; }
};

}

#endif // MDPLIB_CMDPSLACKSOLVER_H