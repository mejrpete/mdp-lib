#ifndef EXTRACTOR_ENGINE_H
#define EXTRACTOR_ENGINE_H

#include "search_engine.h"

// Evaluates all actions by simulating a run that starts with that action
// followed by random actions until a terminal state is reached

class ExtractorEngine : public ProbabilisticSearchEngine {
public:
    ExtractorEngine();

    // Set parameters from command line
    bool setValueFromString(std::string& param, std::string& value) override;

    // Start the search engine to estimate the Q-value of a single action
    void estimateQValue(State const& state, int actionIndex,
                        double& qValue) override;

    // Start the search engine to estimate the Q-values of all applicable
    // actions
    void estimateQValues(State const& state,
                         std::vector<int> const& actionsToExpand,
                         std::vector<double>& qValues) override;

    // Parameter Setter
    virtual void setNumberOfIterations(int _numberOfIterations) {
        numberOfIterations = _numberOfIterations;
    }

private:
    void sampleSuccessorState(PDState const& current, int const& actionIndex,
                              PDState& next, double& reward) const;

    void performRandomWalks(PDState const& root, int firstActionIndex,
                            double& result) const;
    // Parameter
    int numberOfIterations;
};

#endif
