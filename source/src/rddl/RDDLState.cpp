#include "../../include/rddl/RDDLProblem.h"
#include "../../include/rddl/RDDLState.h"

#include <iostream>

namespace mlrddl
{
    std::ostream& RDDLState::print(std::ostream& os) const
    {
        pState_->print(os);
	return os;
    }
}
