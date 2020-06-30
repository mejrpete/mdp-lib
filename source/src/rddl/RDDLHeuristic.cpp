#include "../../include/rddl/RDDLHeuristic.h"
#include "../../include/rddl/RDDLProblem.h"
#include "../../include/rddl/RDDLState.h"

namespace mlrddl
{

    RDDLHeuristic::RDDLHeuristic(RDDLProblem* problem)
    {
    }

    double RDDLHeuristic::cost(const mlcore::State* s)
    {
	//RDDLState* rddlState = (RDDLState *) s;
	//return heuristic value
	return 0;
    }

} // namespace mlrddl
