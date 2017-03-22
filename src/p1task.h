#ifndef P1TASK_H
#define P1TASK_H

#include <mutex>
#include <string>

#include "task.h"
#include "jobserver.h"

class P2Task;

class P1Task: public Task {
  public:
    P1Task(std::string & problem, int objCount, int objCountTotal, int * objectives,
        int numSteps, JobServer *taskServer);

    void addNextLevel(Task * nextLevel);
    Status operator()();

    virtual std::string str() const;
    virtual std::string details() const;

  private:
    /**
     * Number of steps to take along each objective dimension when splitting
     * the objective search space. For instance, a 3-objective problem with 2
     * steps would result in a 2x2 square along one objective, or 4 separate
     * columns to search.
     */
    int numSteps_;

    JobServer * taskServer_;
    std::list<Task *> nextLevel_;
};

inline P1Task::P1Task(std::string & filename, int objCount, int objCountTotal,
    int * objectives, int numSteps, JobServer *taskServer) :
    Task(filename, objCount, objCountTotal, objectives), numSteps_(numSteps),
    taskServer_(taskServer) {

}

inline void P1Task::addNextLevel(Task * nextLevel) {
  nextLevel_.push_back(nextLevel);
}

#endif /* P1TASK_H */