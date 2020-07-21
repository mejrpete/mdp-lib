#ifndef EXTRACTOR_H
#define EXTRACTOR_H

#include <string>
#include <iostream>
#include "states.h"
#include "extractor_engine.h"


namespace rddlmdp
{
    struct SuccessorState {
	State state;
	double prob;
    };

    //wrappers for classes made available as part of the lib that need to be in the namespace
    typedef ActionState ActionState;
    typedef State State;
    typedef PDState PDState;

    class Extractor {

    public:
	Extractor(std::string const& problemName, std::string const& problemDir);
	~Extractor();
	void exploring();

	ActionState* getActionStateRef(int const& actionIndex);
	int nActions();
	void allStateConfigurations(State const& state, ActionState const& action, std::vector<SuccessorState>& successorStates);

	double reward(State const& current, ActionState const& action);

	bool actionIsApplicable(ActionState const& action, State const& state);

	const State& getInitialState();

    private:
	void recursePossibleConfigurations(PDState const& pstate, std::vector<std::vector<double>>& results, std::vector<double> const& partial, int const& index);

	void executeRDDLParser(std::string const& fullProblemPath, std::string const& outPath);
	void executeParser();

	std::string problemDir;
	std::string problemName;

	struct StateActionPairHash {
	    unsigned int operator()(const std::pair<State, ActionState>& sa) const {
		State s = sa.first;
		ActionState a = sa.second;

		State::HashWithoutRemSteps h1;

		int sh = h1(s);

		return (sh + 1571)^a.index; //todo: this should be done in a more rigorous way. sloppy hash function
	    }
	};
	struct StateActionPairEqual {
	    unsigned int operator()(const std::pair<State, ActionState>& lhs, const std::pair<State, ActionState>& rhs) const {
		State sl = lhs.first;
		ActionState al = lhs.second;

		State sr = rhs.first;
		ActionState ar = rhs.second;

		State::EqualWithoutRemSteps e1;

		return (e1(sl, sr) && (al.index == ar.index)); //todo: there should be a better way of checking if actions are equal then relying on index, right? does it matter?
	    }
	};



	std::unordered_map<std::pair<State, ActionState>, std::vector<std::vector<double>>, StateActionPairHash, StateActionPairEqual> cachedConfigs;

	ExtractorEngine se;

    };

}

#endif
