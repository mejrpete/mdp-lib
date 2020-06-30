#ifndef MDPLIB_RDDLHEURISTIC_H
#define MDPLIB_RDDLHEURISTIC_H

#include "../Heuristic.h"
#include "../State.h"

#include "RDDLProblem.h"

namespace mlrddl
{

class RDDLHeuristic : public mlcore::Heuristic
{
private:

public:
    RDDLHeuristic(RDDLProblem* problem);

    virtual double cost(const mlcore::State* s);
};

} // namespace mlrddl

#endif // MDPLIB_RDDLHEURISTIC_H
