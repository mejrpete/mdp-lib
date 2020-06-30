#include "../../include/rddl/RDDLAction.h"

namespace mlrddl
{

std::ostream& RDDLAction::print(std::ostream& os) const
{
    pAction_->printCompact(os);
    return os;
}

}
