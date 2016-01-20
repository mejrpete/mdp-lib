#ifndef METAREASONINGSIMULATOR_H
#define METAREASONINGSIMULATOR_H

#include "../../problem.h"

enum ActionSelectionRule
{
    META_ASSUMPTION_1,
    META_ASSUMPTION_2,
    META_CHANGE_ACTION,
    NO_META,
    QVALIMPROV,
    MULTNOP,
    OPTIMAL
};

namespace mlsolvers
{

/**
 * Class used to simulate an optimal metareasoning algorithm
 * based on Value Iteration.
 * It requires discounting (i.e., gamma < 1.0).
 */
class MetareasoningSimulator
{
private:

    /* The problem to solve. */
    mlcore::Problem* problem_;

    /* A metareasoning problem (only used if the action
                                selection rule is OPTIMAL). */
    mlcore::Problem* metaProblem_;

    /* Residual tolerance used as a stopping condition for VI. */
    double tolerance_;

    /* Expected cost of the policies at each iteration of VI. */
    std::vector< mlcore::StateDoubleMap> policyCosts_;

    /* The estimated values at each iteration of VI. */
    std::vector< mlcore::StateDoubleMap> stateValues_;

    /* Computes the expected cost of VI's current policy. */
    void computeExpectedCostCurrentPolicy(
        mlcore::StateDoubleMap& expectedCosts_);

    /* The number of planning steps per action execution. */
    int numPlanningStepsPerAction_;

    /* The number of planning steps per NOP action. */
    int numPlanningStepsPerNOP_;

    /* Whether to evaluate the current best action against NOP,
    * or all of the actions */
    bool tryAllActions_;

    /* The rule to use to select actions. */
    ActionSelectionRule rule_;

    /* The cost of executing a NOP. */
    double costNOP_;

    /*
     * Assumes that the policy won't change after executing one action (either
     * NOP or the current action). Formally, returns the action chosen using
     * the following Q-values:
     *
     *  Q(s, NOP) = C(s, NOP) + EC[t + dtnop][s]
     *  Q(s, current) = C(s,current) + sum_s' T(s',s,current) EC[t + dta][s']
     *
     *  where:
     *     dtnop := numPlanningStepsPerNOP_OptimalMeta
     *     dta := numPlanningStepsPerAction_
     *     EC := policyCosts_
     *     C(s, NOP) = costNOP_
     *     current := action chosen by the current plan
     *            (computed as in getActionNoMetareasoning)
     *
     * If Q(s, NOP) < Q(s, current), NOP is returned, otherwise a is returned.
     *
     * If the "tryAllActions_" variable is set to TRUE, then instead of using
     * the action chosen by the current plan, the method will look for the
     * action with the lowest Q(s,current) as computed above.
     */
    mlcore::Action* getActionMetaAssumption1(mlcore::State* s, int t);

    /*
     * Similar to getActionMetaAssumption1, except that instead of allowing a
     * single NOP, a sequence of multiple NOPs can be chosen as a metareasoning
     * action. Thus, the metareasoning choices are:
     *
     *   -A sequence of multiple NOPs.
     *   -The current best action recommended by the plan.
     *
     * After either of these actions is chosen, it is assumed that the policy
     * will remain fixed thereafter.
     *
     * The length of the sequence of NOPs is chosen as the minimum of:
     *   1) The number of NOPs so that there are no more policy improvements.
     *   2) The number of NOPs so that the cost of the NOPs plus the cost of
     *      the resulting policy is better than Q(s, current), as defined in
     *      getActionMetaAssumption1.
     */
    mlcore::Action*
    getActionMetaAssumption1MultipleNOPs(mlcore::State* s, int t);

    /*
     * Returns the action chosen using the following Q-values:
     *
     *  Q(s, NOP) = C(s, NOP) + V*[s]
     *  Q(s, a) = C(s,a) + sum_s' T(s',s,a) V*[s']
     *
     *  where:
     *     V*[s] := optimal value for the state s
     *     C(s, NOP) = costNOP_
     *     a := action chosen by the current plan
     *            (computed as in getActionNoMetareasoning)
     *
     * If Q(s, NOP) < Q(s, a), NOP is returned, otherwise a is returned.
     */
    mlcore::Action* getActionMetaAssumption2(mlcore::State* s, int t);

    /*
     * Returns the action chosen as follows:
     *
     * For any action a we define:
     *  Q*(s, a) = C(s,a) + sum_s' T(s',s,a) V*[s']
     *
     * Let a*, ac, and anop, be the optimal action, the current action and the
     * action recommended by NOP, respectively.
     *
     * Then:
     *
     *    If Q*(s, ac) == Q*(s, a*), return ac
     *    Else
     *      If Q*(s, anop) + C(s, NOP) < Q*(s. ac), return NOP
     *      Else, return a
     */
    mlcore::Action* getActionQValueImprovement(mlcore::State* s, int t);

    /*
     * Returns the action chosen using the Values estimated by VI at each time
     * step (that is, the current best plan):
     *  action = arg min_{a \in A} C(s,a) + sum_s' T(s',s,a) EC[t][s']
     */
    mlcore::Action* getActionNoMetareasoning(mlcore::State* s, int t);

    /*
     * Returns the action chosen according to the following rule:
     *     If the action chosen by the planner would change after one
     *     application of NOP, then use NOP. Otherwise, stick to the current
     *     action.
     */
    mlcore::Action* getActionMetaChangeBestAction(mlcore::State* s, int t);

    /*
     * Returns the action chosen accordint the optimal metareasoning problem
     * as described in the MetareasoningProblem class.
     */
    mlcore::Action* getActionOptimalMetareasoning(mlcore::State* s, int t);

    /*
     * Pre-computes and stores the expected costs
     * of all the intermediate policies found during Value Iteration.
     */
    void precomputeAllExpectedPolicyCosts();

    /*
     * Samples the expected cost of reaching the goal from the given state
     * using the policy implicit at time t.
     */
    double
    estimateExpectedCostPolicyAtTime(mlcore::State* s, int t, int trials);

public:
    MetareasoningSimulator(mlcore::Problem* problem,
                            double tolerance = 1.0e-6,
                            int numPlanningStepsPerAction = 5,
                            int numPlanningStepsPerNOP = 5,
                            double costNOP = 1.0,
                            bool tryAllActions = false,
                            ActionSelectionRule rule = META_ASSUMPTION_1);

    virtual ~MetareasoningSimulator()
    {
        if (metaProblem_ != nullptr)
            delete metaProblem_;
    }

    mlcore::Problem* problem() { return problem_; }

    double tolerance() { return tolerance_; }

    std::vector< mlcore::StateDoubleMap>&
        stateValues() { return stateValues_; };

    void rule(ActionSelectionRule value);

    void tryAllActions(bool value) { tryAllActions_ = value; }

    int numPlanningStepsPerAction() { return numPlanningStepsPerAction_; }

    void numPlanningStepsPerAction(int value)
        { numPlanningStepsPerAction_ = value; }

    int numPlanningStepsPerNOP() { return numPlanningStepsPerNOP_; }

    void numPlanningStepsPerNOP(int value) { numPlanningStepsPerNOP_ = value; }

    double costNOP() { return costNOP_; }

    void costNOP(double value) { costNOP_ = value; }

    /**
     * Simulates a run of a metareasoning approach that uses the information
     * collected by precomputeAllExpectedPolicyCosts.
     *
     * The method returns a pair with the cost accumulated during the
     * simulation and the total cost spent doing NOPs.
     */
    std::pair<double, double> simulate();
};

} // mlsolvers


#endif // METAREASONINGSIMULATOR_H
