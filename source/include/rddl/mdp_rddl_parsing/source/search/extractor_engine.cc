#include "extractor_engine.h"

/******************************************************************
                     Search Engine Creation
******************************************************************/

ExtractorEngine::ExtractorEngine() :
    ProbabilisticSearchEngine("ExtractorEngine"),
    numberOfIterations(1) {}

bool ExtractorEngine::setValueFromString(std::string& param,
                                    std::string& value) {

    if (param == "-it") {
        setNumberOfIterations(atoi(value.c_str()));
        return true;
    }

    return SearchEngine::setValueFromString(param, value);
}

void ExtractorEngine::estimateQValue(State const& state, int actionIndex,
                                double& qValue) {
    assert(state.stepsToGo() > 0);
    PDState current(state);
    performRandomWalks(current, actionIndex, qValue);
}

void ExtractorEngine::estimateQValues(State const& state,
                                 std::vector<int> const& actionsToExpand,
                                 std::vector<double>& qValues) {
    assert(state.stepsToGo() > 0);
    PDState current(state);
    for (size_t index = 0; index < qValues.size(); ++index) {
        if (actionsToExpand[index] == index) {
            performRandomWalks(current, index, qValues[index]);
        }
    }
}

void ExtractorEngine::performRandomWalks(PDState const& root, int firstActionIndex,
                                    double& result) const {
    result = 0.0;
    double reward = 0.0;
    PDState next;

    for (unsigned int i = 0; i < numberOfIterations; ++i) {
        PDState current(root.stepsToGo() - 1);
        sampleSuccessorState(root, firstActionIndex, current, reward);
        result += reward;

        while (current.stepsToGo() > 0) {
            int rndActionIndex = MathUtils::rnd->randomElement(
                getIndicesOfApplicableActions(current));
            next.reset(current.stepsToGo() - 1);
            sampleSuccessorState(current, rndActionIndex, next, reward);
            result += reward;
            current = next;
        }
    }
    result /= (double)numberOfIterations;
}


/******************************************************************
                       Main Search Functions
******************************************************************/

void ExtractorEngine::sampleSuccessorState(PDState const& current,
                                      int const& actionIndex, PDState& next,
                                      double& reward) const {
    calcReward(current, actionIndex, reward);
    calcSuccessorState(current, actionIndex, next);
    for (unsigned int varIndex = 0;
         varIndex < State::numberOfProbabilisticStateFluents; ++varIndex) {
        next.sample(varIndex);
    }
}