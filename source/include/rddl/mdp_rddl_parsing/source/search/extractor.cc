#include "extractor.h"
#include "parser.h"
#include "utils/system_utils.h"

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

namespace rddlmdp {

    Extractor::Extractor(std::string const& problemName, std::string const& problemDir) {
	this->problemName = problemName;
	this->problemDir = problemDir;
	this->executeParser();
	ExtractorEngine se;

    }


    Extractor::~Extractor() {
	fs::remove(problemDir + problemName);
	fs::remove(problemDir + problemName + ".json");
    }

    void Extractor::executeRDDLParser(std::string const& fullProblemPath, std::string const& outPath) {
	/// RDDL-PARSER

	// rddl-parser cmd
	std::string rddl_parser_cmd = "";
	rddl_parser_cmd += "./rddl-parser ";
	rddl_parser_cmd += fullProblemPath + " ";
	rddl_parser_cmd += outPath;

	std::cout << rddl_parser_cmd << std::endl;

	rddl_parser_cmd +=
	    " -outcome_pruning " + std::to_string(0.5);

	int return_val = system(rddl_parser_cmd.c_str());
	if (return_val != 0) {
	    std::cout << "RDDL-Parser error!" << std::endl;
	} else {
	    std::cout << "Successfully parsed RDDL" << std::endl;
	}
    }

    void Extractor::executeParser() {

	std::map<std::string, int> stateVariableIndices;
	std::vector<std::vector<std::string>> stateVariableValues;

	// Assumes that rddl-parser executable exists in the current directory.
	std::cout << fs::current_path() << std::endl;
	if (!fs::exists(fs::current_path() / "rddl-parser")) {
	    SystemUtils::abort(
			       "Error: rddl-parser executable not found in working directory.");
	}
	//TODO: don't know why this doesn't work. need to figure this out eventually
	//check to make sure the requested instance file actually exists
	//    if (!fs::exists( fs::current_path() / problemDir / (problemName + ".rddl"))) {
	//        SystemUtils::abort(
	//            "Error: Input RDDL file not found at the specified path.");
	//    }

	executeRDDLParser(problemDir + problemName + ".rddl", problemDir);

	Parser parser(problemDir + problemName);
	parser.parseTask(stateVariableIndices, stateVariableValues);

	// Remove temporary files
	fs::remove(problemDir + problemName);
    }

    ActionState* Extractor::getActionStateRef(int const& actionIndex) {
	return &se.actionStates[actionIndex];
    }

    int Extractor::nActions() {
	return se.actionStates.size();
    }

    const State& Extractor::getInitialState() {
	return SearchEngine::initialState;
    }

    //this could be done more efficiently if joint prob calc was
    //integrated with state generation. (just a constant factor though.)
    //todo: this ignores possible state invariants which constrain things. is this a problem?
    void Extractor::allStateConfigurations(State const& state,
				      ActionState const& action,
				      std::vector<SuccessorState>& successorStates) {

	assert(se.actionIsApplicable(action, state));
	int actionIndex = action.index;

	//we set up PDState with the correct properties to operate as a prob. successor
	//this includes assigning each of the determiniistic CPF results
	//and setting the probabilisticstatefluentaspd entries for the successor
	PDState next;
	se.calcSuccessorState(state, actionIndex, next);

	//we get all permutations of state values. this should be cached but isn't currently
	//TODO: cache these.
	std::vector<std::vector<double>> results;
	std::pair<State, ActionState> id (state, action);
	//	if (cachedConfigs.find(id) == cachedConfigs.end()) {
	    std::vector<double> partial;
	    recursePossibleConfigurations(next, results, partial, 0);
	    //cachedConfigs[id] = results;
	    //	}
	    //	else {
	    //	    results = cachedConfigs.at(id);
	    //	}

	for (int i = 0; i < results.size(); ++i) {
	    std::vector<double> deterministic(next.numberOfDeterministicStateFluents, -1);
	    for (int d = 0; d < next.numberOfDeterministicStateFluents; ++d) {
		deterministic.at(d) = next.deterministicStateFluent(d);
	    }
	    std::vector<double> probabilitistic(next.numberOfProbabilisticStateFluents, -1);
	    double prob = 1;
	    double val;
	    for (int p = 0; p < next.numberOfProbabilisticStateFluents; ++p) {
		//val = next.probabilisticStateFluent(p);
		val = results.at(i).at(p);
		probabilitistic.at(p) = val;
		prob *= next.probabilisticStateFluentAsPD(p).probabilityOf(val);
	    }
	    State s(deterministic, probabilitistic, state.stepsToGo()-1); //TODO: should deal with this some other way. we're relying on the caller to destroy these
	    SuccessorState toAdd;
	    toAdd.state = s;
	    toAdd.prob = prob;
	    successorStates.push_back(toAdd);
	}
    }
    
    void Extractor::recursePossibleConfigurations(PDState const& pstate, std::vector<std::vector<double>>& results, std::vector<double> const& partial, int const& index) {
	if (index >= pstate.numberOfProbabilisticStateFluents) {
	    std::vector<double> toAdd = partial;
	    results.push_back(toAdd);
	}
	else {
	    DiscretePD fluentDistribution = pstate.probabilisticStateFluentAsPD(index);
	    for (int i = 0; i < fluentDistribution.getNumberOfOutcomes(); ++i) {
		std::vector<double> appended = partial;
		appended.push_back(fluentDistribution.values[i]);
		recursePossibleConfigurations(pstate, results, appended, index + 1);
	    }
	}
	return;
    }

    double Extractor::reward(State const& current, ActionState const& action) {
	double res;
	se.rewardCPF->evaluate(res, current, action);
	return res;
    }

    bool Extractor::actionIsApplicable(ActionState const& action,
				       State const& state) {
	return se.actionIsApplicable(action, state);
    }

    void Extractor::exploring() {
	State cur = SearchEngine::initialState;

	std::cout << "initial state: " << std::endl;
	cur.print(std::cout);
	std::vector<int> actions = se.getApplicableActions(cur);
	for (auto& a: actions) {
	    cur = SearchEngine::initialState;
	    PDState next;
	    if (a != -1) {
		se.calcSuccessorState(cur, a, next);
		std::cout << "new state: " << std::endl;
		next.print(std::cout);
	    }
	}
	for (auto& a : se.actionStates) {
	    //////////////////	a.print(cout);
	    for (auto& sch : a.scheduledActionFluents) {
		sch->print(std::cout);
	    }
	    std::cout << std::endl;
	}
	std::cout << "this is a test" << std::endl;
    }

}
