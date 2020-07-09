#include <sstream>

#include "../../include/rddl/RDDLAction.h"
#include "../../include/rddl/RDDLProblem.h"
#include "../../include/rddl/RDDLState.h"


namespace mlrddl
{

    RDDLProblem::RDDLProblem(rddlmdp::Extractor* problemExtractor) : problemExtractor_(problemExtractor)
{
    //Get initial state
    //Add initial state to this problem
    rddlmdp::State* persistBackendS0 = new rddlmdp::State(problemExtractor_->getInitialState());
    s0 = new RDDLState((mlcore::Problem*)this, persistBackendS0);
    this->addState(s0);

    //get all actions and push back new RDDLActions
    for (int i = 0; i < problemExtractor_->nActions(); ++i) {
	actions_.push_back(new RDDLAction(problemExtractor_->getActionStateRef(i), i));
    }
}


bool RDDLProblem::goal(mlcore::State* s) const
{
    //fake goal based on the horizon
    //return if you're there
    RDDLState* state = (RDDLState *) s;
    return state->pState()->isTerminal();//todo: would like to have a more sophisticated understanding of goals, but that requires parser mods
}


std::list<mlcore::Successor>
    RDDLProblem::transition(mlcore::State* s, mlcore::Action* a)
{
    assert(applicable(s,a));
    RDDLAction* action = (RDDLAction *) a;
    RDDLState* state = (RDDLState *) s;

    std::vector<rddlmdp::SuccessorState> allSuccessors;
    problemExtractor_->allStateConfigurations(*state->pState(), *action->pAction(), allSuccessors);
    
    std::list<mlcore::Successor> successors;
    //for the state given the action, build a Successor list with
    //the new state and transition probability
    //TODO: are we supposed to be including successors with prob 0?
    for (int i = 0; i < allSuccessors.size(); ++i) {
	if (allSuccessors[i].prob > 0.00000000000001) { //TODO: ... deal with this constant
	    rddlmdp::State* persistBackendS = new rddlmdp::State(allSuccessors[i].state);
	    RDDLState* nextState = new RDDLState(this, persistBackendS);
	    assert(persistBackendS->stepsToGo() == state->pState()->stepsToGo() -1);
	    successors.push_back(mlcore::Successor(this->addState(nextState), allSuccessors[i].prob));
	}
    }
    return successors;
}


double RDDLProblem::cost(mlcore::State* s, mlcore::Action* a) const
{
    RDDLAction* action = (RDDLAction *) a;
    RDDLState* state = (RDDLState *) s;

    double cost = problemExtractor_->reward(*state->pState(), *action->pAction());
    assert(cost <= 0);
    return -cost;
}


bool RDDLProblem::applicable(mlcore::State* s, mlcore::Action* a) const
{
    RDDLAction* action = (RDDLAction *) a;
    RDDLState* state = (RDDLState *) s;

    return problemExtractor_->actionIsApplicable(*action->pAction(), *state->pState());
}
}
