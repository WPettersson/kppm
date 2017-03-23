/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef P1TASK_H
#define P1TASK_H

#include <mutex>
#include <string>

#include "sense.h"
#include "task.h"
#include "jobserver.h"

class P2Task;

class P1Task: public Task {
  public:
    P1Task(std::string & problem, int objCount, int objCountTotal, Sense sense,
        int * objectives, int numSteps, JobServer *taskServer);

    void addNextLevel(Task * nextLevel);
    Status operator()();

    virtual std::string str() const;
    virtual std::string details() const;

  private:
    Sense sense_;
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
    Sense sense, int * objectives, int numSteps, JobServer *taskServer) :
    Task(filename, objCount, objCountTotal, objectives), sense_(sense),
    numSteps_(numSteps), taskServer_(taskServer) {

}

inline void P1Task::addNextLevel(Task * nextLevel) {
  nextLevel_.push_back(nextLevel);
}

#endif /* P1TASK_H */
