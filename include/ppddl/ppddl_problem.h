#ifndef MDPLIB_PPDDLPROBLEM_H
#define MDPLIB_PPDDLPROBLEM_H

#include "../problem.h"
#include "mini-gpt/problems.h"
#include "mini-gpt/rational.h"

namespace mlppddl
{

typedef std::pair<state_t *, Rational> successor_t;

/**
 * A class representing a PPDDL problem. The implementation is based on
 * the mini-gpt library (see http://ldc.usb.ve/~bonet/reports/JAIR-mgpt.pdf).
 */
class Problem : public mlcore::Problem
{
private:
    problem_t* pProblem_;

    successor_t display_[DISP_SIZE];

public:
    Problem(problem_t* pProblem);

    virtual ~Problem()
    {
        delete pProblem_;
    }

    problem_t* pProblem() { return pProblem_; }

    /**
     * Overrides method from Problem.
     */
    virtual bool goal(mlcore::State* s) const;

    /**
     * Overrides method from Problem.
     */
    virtual std::list<mlcore::Successor> transition(mlcore::State* s,
                                                    mlcore::Action* a);

    /**
     * Overrides method from Problem.
     */
    virtual double cost(mlcore::State* s, mlcore::Action* a) const;

    /**
     * Overrides method from Problem.
     */
    virtual bool applicable(mlcore::State* s, mlcore::Action* a) const;
};

}

#endif // MDPLIB_PPDDLPROBLEM_H
