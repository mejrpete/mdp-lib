#ifndef MDPLIB_RDDLPROBLEM_H
#define MDPLIB_RDDLPROBLEM_H

#include "mdp_rddl_parsing/source/search/extractor.h"

#include "../Problem.h"

namespace mlrddl
{

/**
 * A class representing a RDDL problem. The implementation uses parsing and representation
 * code from the PROST planner (TODO: link to prost planner in docs)
 */
class RDDLProblem : public mlcore::Problem
{
private:
    rddlmdp::Extractor* problemExtractor_;

public:
    RDDLProblem(rddlmdp::Extractor* problemExtractor);

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

#endif // MDPLIB_RDDLPROBLEM_H
