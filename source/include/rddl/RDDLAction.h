#ifndef MDPLIB_RDDLACTION_H
#define MDPLIB_RDDLACTION_H

#include "mdp_rddl_parsing/source/search/extractor.h"

#include "../Action.h"

namespace mlrddl
{

class RDDLAction : public mlcore::Action
{
private:
    int index_;
    const rddlmdp::ActionState* pAction_;

    virtual std::ostream& print(std::ostream& os) const;

public:
    RDDLAction(const rddlmdp::ActionState* pAction, int index) :
        pAction_(pAction), index_(index) {}

    const rddlmdp::ActionState* pAction() { return pAction_; }

    /**
     * Overriding method from Action.
     */
    virtual mlcore::Action& operator=(const mlcore::Action& rhs)
    {
        if (this == &rhs)
            return *this;

        RDDLAction* action = (RDDLAction*)  & rhs;
        pAction_ = action->pAction_;
        index_ = action->index_;
        return *this;
    }

    /**
     * Overriding method from Action.
     */
    virtual int hashValue() const
    {
        return index_;
    }
};

}

#endif // MDPLIB_RDDLACTION_H
