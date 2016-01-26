#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../../../include/problem.h"
#include "../../../include/domains/gridworld/GridWorldProblem.h"
#include "../../../include/domains/gridworld/GridWorldState.h"
#include "../../../include/domains/gridworld/GridWorldAction.h"

void GridWorldProblem::addAllActions()
{
    mlcore::Action* up = new GridWorldAction(gridworld::UP);
    mlcore::Action* down = new GridWorldAction(gridworld::DOWN);
    mlcore::Action* left = new GridWorldAction(gridworld::LEFT);
    mlcore::Action* right = new GridWorldAction(gridworld::RIGHT);
    actions_.push_front(up);
    actions_.push_front(down);
    actions_.push_front(left);
    actions_.push_front(right);
}

GridWorldProblem::GridWorldProblem() :
                    width_(0), height_(0), x0_(0), y0_(0),
                    goals_(0), actionCost_(0.03)
{
    absorbing = new GridWorldState(this, -1, -1);
    addAllActions();
}

GridWorldProblem::GridWorldProblem(
    const char* filename, PairDoubleMap* goals, double actionCost)
{
    mdplib_debug = true;
    std::ifstream myfile (filename);

    // Once the file is finished parsing, these will have correct values
    width_ = 0, height_ = 0;
    if (myfile.is_open()) {
        std::string line;
        while ( std::getline (myfile, line) ) {
            for (width_ = 0; width_ < line.size(); width_++) {
                if (line.at(width_) == 'x') {
                    walls.insert(std::pair<int, int>(width_, height_));
                } else if (line.at(width_) == 'G') {
                    goals->insert(
                        std::make_pair(
                            std::pair<int,int> (width_, height_), 0.0));
                } else if (line.at(width_) == 'S') {
                    x0_ = width_;
                    y0_ = height_;
                } else {
                    assert(line.at(width_) == '.');
                }
            }
            height_++;
        }
        myfile.close();
    }
    goals_ = goals;
    actionCost_ = actionCost;
    s0 = new GridWorldState(this, x0_, y0_);
    absorbing = new GridWorldState(this, -1, -1);
    this->addState(s0);
    this->addState(absorbing);
    addAllActions();
}

GridWorldProblem::GridWorldProblem(
    int width, int height, int x0, int y0,
    PairDoubleMap* goals, double actionCost) :
        width_(width), height_(height), x0_(x0), y0_(y0),
        goals_(goals), actionCost_(actionCost)
{
    s0 = new GridWorldState(this, x0_, y0_);
    absorbing = new GridWorldState(this, -1, -1);
    this->addState(s0);
    addAllActions();
}

GridWorldProblem::GridWorldProblem(int width, int height,
                                   int x0, int y0,
                                   PairDoubleMap* goals, mlcore::Heuristic* h)
                                   : width_(width), height_(height),
                                      x0_(x0), y0_(y0), goals_(goals)
{
    s0 = new GridWorldState(this, x0_, y0_);
    absorbing = new GridWorldState(this, -1, -1);
    this->addState(s0);
    heuristic_ = h;
    gamma_ = 1.0;
    addAllActions();
}

bool GridWorldProblem::gridGoal(mlcore::State* s) const
{
    GridWorldState* gws = (GridWorldState *) s;
    std::pair<int,int> pos(gws->x(),gws->y());
    return goals_->find(pos) != goals_->end();
}

bool GridWorldProblem::goal(mlcore::State* s) const
{
    return s == absorbing;
}

std::list<mlcore::Successor>
GridWorldProblem::transition(mlcore::State *s, mlcore::Action *a)
{
    GridWorldState* state = (GridWorldState *) s;
    GridWorldAction* action = (GridWorldAction *) a;

    std::list<mlcore::Successor> successors;

    if (s == absorbing || gridGoal(s)) {
        successors.push_front(mlcore::Successor(absorbing, 1.0));
        return successors;
    }

    if (action->dir() == gridworld::UP) {
        addSuccessor(state, successors, height_ - 1, state->y(),
                     state->x(), state->y() + 1, 0.8);

        addSuccessor(state, successors, state->x(), 0,
                     state->x() - 1, state->y(), 0.1);

        addSuccessor(state, successors, width_ - 1, state->x(),
                     state->x() + 1, state->y(), 0.1);
    } else if (action->dir() == gridworld::DOWN) {
        addSuccessor(state, successors, state->y(), 0,
                     state->x(), state->y() - 1, 0.8);

        addSuccessor(state, successors, state->x(), 0,
                     state->x() - 1, state->y(), 0.1);

        addSuccessor(state, successors, width_ - 1, state->x(),
                     state->x() + 1, state->y(), 0.1);
    } else if (action->dir() == gridworld::LEFT) {
        addSuccessor(state, successors, state->x(), 0,
                     state->x() - 1, state->y(), 0.8);

        addSuccessor(state, successors, state->y(), 0,
                     state->x(), state->y() - 1, 0.1);

        addSuccessor(state, successors, height_ - 1, state->y(),
                     state->x(), state->y() + 1, 0.1);
    } else if (action->dir() == gridworld::RIGHT) {
        addSuccessor(state, successors, width_ - 1, state->x(),
                     state->x() + 1, state->y(), 0.8);

        addSuccessor(state, successors, state->y(), 0,
                     state->x(), state->y() - 1, 0.1);

        addSuccessor(state, successors, height_ - 1, state->y(),
                     state->x(), state->y() + 1, 0.1);
    }
    return successors;
}

double GridWorldProblem::cost(mlcore::State* s, mlcore::Action* a) const
{
    if (s == absorbing)
        return 0.0;
    if (gridGoal(s)) {
        GridWorldState* gws = (GridWorldState *) s;
        std::pair<int,int> pos(gws->x(),gws->y());
        return (*goals_)[pos];
    }
    return actionCost_;
}

bool GridWorldProblem::applicable(mlcore::State* s, mlcore::Action* a) const
{
    return true;
}

void GridWorldProblem::addSuccessor(
    GridWorldState* state, std::list<mlcore::Successor>& successors,
    int val, int limit, int newx, int newy, double prob)
{
    bool isWall = (walls.count(std::pair<int, int> (newx, newy)) != 0);
    if (val > limit && !isWall) {
        GridWorldState *next = new GridWorldState(this, newx, newy);
        successors.push_front(mlcore::Successor(this->addState(next), prob));
    } else {
        successors.push_front(mlcore::Successor(state, prob));
    }
}
