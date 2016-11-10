#ifndef MDPLIB_FFUTIL_H
#define MDPLIB_FFUTIL_H

/**
 * This file provides some functions that are useful to interact with the
 * FF planner.
 */

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <unordered_set>

#include "../ppddl/PPDDLProblem.h"
#include "../ppddl/PPDDLState.h"


namespace mlsolvers
{


/*
 * Checks the init state in the given PPDDL problem filename and stores the
 * atoms that are are not part of the given PPDDL problem object
 * (problem_t type) atom_hash.
 * The set of atoms is returned as a string.
 */
inline std::string storeRemovedInitAtoms(
    std::string problemFilename, mlppddl::PPDDLProblem* problem)
{
    // Storing all atoms in the initial state
    std::ifstream problemFile;
    problemFile.open(problemFilename, std::ifstream::in);
    std::string line;
    std::unordered_set<std::string> initAtoms;
    if (problemFile.is_open()) {
        while (getline(problemFile, line)) {
            size_t idx = line.find("init");
            if (idx != std::string::npos) {
                for (int i = idx + 4; i < line.size(); i++) {
                    if (line[i] == '(') {
                        std::string atom = "";
                        do {
                            atom += line[i];
                        } while (line[i++] != ')');
                        initAtoms.insert(atom);
                        i--;
                    }
                }
            }
        }
        problemFile.close();
    }

    // Figuring out which atoms were removed from the PPDDL parser
    problem_t* pProblem = problem->pProblem();
    Domain dom = pProblem->domain();
    PredicateTable& preds = dom.predicates();
    TermTable& terms = pProblem->terms();
    for (auto const & atom : problem_t::atom_hash()) {
        std::ostringstream oss;
        atom.first->print(oss, preds, dom.functions(), terms);
        if (initAtoms.find(oss.str()) != initAtoms.end())
            initAtoms.erase(oss.str());
    }

    // Storing the removed atoms
    std::string removedInitAtoms = "";
    for (std::string atom : initAtoms)
        removedInitAtoms += atom + " ";
    return removedInitAtoms;
}


/**
 * Extracts the atoms in the given PPDDL state and returns them as a string.
 */
inline std::string extractStateAtoms(mlppddl::PPDDLState* state)
{
    std::ostringstream oss;
    oss << state;
    std::string stateStr = oss.str();
    std::string atomsCurrentState = "";
    for (int i = 0; i < stateStr.size(); i++) {
        // The format of a PPDDLState "tostring" conversion is
        // [ atom_1_Id:(atom_1),
        //   atom_2_Id:(atom_2),
        //   ...,
        ///  atom_N_Id:(atom_N) ]
        if (stateStr[i] == ':') {
            i++;
            do {
                atomsCurrentState += stateStr[i];
            } while (stateStr[i++] != ')');
            atomsCurrentState += " ";
        }
    }
    return atomsCurrentState;
}


/**
 * Replaces the initial state in the given template PPDDL filename
 * with the given state atoms. Then writes the output to the given
 * output file.
 */
inline void replaceInitStateInProblemFile(
    std::string templateProblemFilename_,
    std::string atomsCurrentState,
    std::string outputProblemFile)
{
    std::ifstream problemTemplateFile;
    problemTemplateFile.open(templateProblemFilename_, std::ifstream::in);
    std::string line;
    std::string newProblemText = "";
    if (problemTemplateFile.is_open()) {
        while (getline(problemTemplateFile, line)) {
            if (line.find("init") != std::string::npos) {
                line = "(:init " + atomsCurrentState + ")";
            }
            newProblemText += line + "\n";
        }
        problemTemplateFile.close();
    }
    std::ofstream newProblemFile;
    newProblemFile.open(outputProblemFile);
    newProblemFile << newProblemText;
    newProblemFile.close();
}


/* Handler for the child process running FF. */
static void sigchld_hdl(int sig)
{
    // the main process will handle the child, no need to do anything
}


/**
 * Runs the FF planner and returns the action name and cost.
 *
 * @param ffExecFilename The name of the executable file for FF.
 * @param determinizedDomainFilename The name of the file where the
 *                                   determinized PPDDL domain is stored.
 * @param currentProblemFilename The name of the file where the PPDDL problem
 *                               is stored.
 * @param startingPlanningTime The time stamp at which the time for planning
 *                             started. Used if a planning limit is desired.
 * @param maxPlanningTime The maximum time allowed for planning, counting from
 *                        startingPlanningTime.
 *
 * @return A pair storing the action name and the computed cost.
 */
inline std::pair<std::string, int> getActionNameAndCostFromFF(
    std::string ffExecFilename,
    std::string determinizedDomainFilename,
    std::string currentProblemFilename,
    int startingPlanningTime = 0,
    int maxPlanningTime = 1000000)
{
    pid_t child_pid;
    int fds[2];
    int pipe_ret = pipe(fds);
    if (pipe_ret != 0) {
        std::cerr << "Error creating pipe for FF: " <<
            strerror(errno) << std::endl;
        exit(-1);
    }

    std::string actionName = "__mdplib-dead-end__";
    int costFF = floor(mdplib::dead_end_cost);
    time_t timeLeft = 0.0;
    if (timeHasRunOut(startingPlanningTime, maxPlanningTime, &timeLeft)) {
        return std::make_pair(actionName, costFF);
    }

    // setting a handler for the FF child process
    struct sigaction act;
    memset (&act, 0, sizeof(act));
    act.sa_handler = sigchld_hdl;
    if (sigaction(SIGCHLD, &act, 0) == -1) {
        std::cerr << "Error setting handler for FF process signal" << std::endl;
        exit(-1);
    }

    child_pid = fork();
    if (child_pid != 0) {   // parent process (process FF output)
        close(fds[1]);
        int status;
        while (timeLeft > 0) {  // TODO: improve this ugly code hack
            timeHasRunOut(startingPlanningTime, maxPlanningTime, &timeLeft);
            pid_t wait_result = waitpid(child_pid, &status, WNOHANG);
            if (wait_result == -1) {
                std::cerr << "Error ocurred during call to FF: " <<
                    strerror(errno) << std::endl;
                exit(-1);
            } else if (wait_result == 0) {  // FF still running
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } else {    // FF finished
                break;
            }
        }
        kill(child_pid, SIGTERM); // seems to be safe to use on child processes
        pid_t wait_result = waitpid(child_pid, &status, 0);
        FILE* ff_output = fdopen(fds[0], "r");
        if (ff_output) {
            char lineBuffer[1024];
            int currentLineAction = -1;
            while (fgets(lineBuffer, 1024, ff_output)) {
                if (strstr(lineBuffer, "goal can be simplified to FALSE.") !=
                        nullptr) {
                    break;
                }
                if (strstr(lineBuffer, "step") != nullptr) {
                    actionName = "";
                    char *pch = strstr(lineBuffer, "0:");
                    if (pch == nullptr)
                        continue;
                    pch += 3;
                    actionName += pch;
                    actionName = actionName.substr(0, actionName.size() - 1);
                    currentLineAction = 0;
                } else if (currentLineAction != -1) {
                    currentLineAction++;
                    std::ostringstream oss("");
                    oss << currentLineAction << ":";
                    char *pch = strstr(lineBuffer, oss.str().c_str());
                    if (pch == nullptr) {
                        costFF = currentLineAction;
                        currentLineAction = -1;
                    }
                }
            }
            pclose(ff_output);
        } else {
            std::cerr << "Error reading the output of FF." << std::endl;
            exit(-1);
        }
        for (int i = 0; i < actionName.size(); i++) {
            actionName[i] = tolower(actionName[i]);
        }
        return std::make_pair(actionName, costFF);
    } else {    // child process (the one that calls FF)
        close(fds[0]);
        dup2(fds[1], STDOUT_FILENO);
        const char* ff_args[] = {
            "ff",
            "-o",
            determinizedDomainFilename.c_str(),
            "-f",
            currentProblemFilename.c_str(),
            NULL
        };
        execvp(ffExecFilename.c_str(), const_cast<char**> (ff_args));
        std::cerr << "An error ocurred while calling FF: " <<
            strerror(errno) << std::endl;
        abort();
    }
}


};

#endif // MDPLIB_FFUTIL_H
