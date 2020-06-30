#ifndef MDPLIB_RDDLSTATE_H
#define MDPLIB_RDDLSTATE_H

#include "mdp_rddl_parsing/source/search/extractor.h"

#include "../State.h"

namespace mlrddl
{

class RDDLState : public mlcore::State
{
private:
    rddlmdp::State* pState_;

    virtual std::ostream& print(std::ostream& os) const;

public:

    RDDLState(mlcore::Problem* problem)
    {
        mlcore::State::problem_ = problem;
	pState_ = new rddlmdp::State;
    }

    RDDLState(mlcore::Problem* problem, rddlmdp::State* pState) : pState_(pState)
    {
        mlcore::State::problem_ = problem;
    }

    virtual ~RDDLState()
    {
        delete pState_;
    }

    rddlmdp::State* pState() { return pState_; }

    void setPState(rddlmdp::State & pState) { *pState_ = pState; }

    /**
     * Overrides method from State.
     */
    virtual mlcore::State& operator=(const mlcore::State& rhs)
    {
        if (this == &rhs)
            return *this;

        RDDLState* state = (RDDLState*)  & rhs;
        pState_ = state->pState_;
        problem_ = state->problem_;
        return *this;
    }

    /**
     * Overrides method from State.
     */
    virtual bool operator==(const mlcore::State& rhs) const
    {
        RDDLState* state = (RDDLState*)  & rhs;

	//get the relevant functor for State equality and apply it.
	rddlmdp::State::EqualWithRemSteps eq; //TODO: should we be doing this with or without remaining steps? Are we caching? Is that what we're using this for?
	return eq(*pState_, *state->pState_);
    }

    /**
     * Overrides method from State.
     */
    virtual bool equals(mlcore::State* rhs) const
    {
        RDDLState* state = (RDDLState*)  & rhs;
        return *this == *rhs;
    }

    /**
     * Overrides method from State.
     */
    virtual int hashValue() const
    {
	rddlmdp::State::HashWithRemSteps h; //TODO: same question as "==", should we be including remSteps?
	return h(*pState_);
    }

};
}

#endif // MDPLIB_RDDLSTATE_H
