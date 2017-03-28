/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef P3CREATOR_H
#define P3CREATOR_H

#ifdef DEBUG
#include <mutex>
#endif

#include "task.h"
#include "env.h"
#include "problem.h"
#include "jobserver.h"

class P3Creator : public Task {
  public:
    P3Creator(std::string & filename, int objCount,
        int objCountTotal, int * objectives, Sense sense,
        double * lower, double * upper,
        JobServer * taskServer);
    Status operator()();

    void addNextLevel(Task * nextLevel);

    virtual std::string str() const;
    virtual std::string details() const;
  private:
    JobServer * taskServer_;
    std::list<Task *> nextLevel_;

    double * lower_;
    double * upper_;
};

inline P3Creator::P3Creator(std::string & filename, int objCount,
    int objCountTotal, int * objectives, Sense sense,
    double * lower, double * upper,
    JobServer * taskServer) :
    Task(filename, objCount, objCountTotal, objectives, sense),
    taskServer_(taskServer) {
  lower_ = new double[objCount_];
  upper_ = new double[objCount_];
  for (int d = 0; d < objCount_; ++d) {
    lower_[d] = lower[d];
    upper_[d] = upper[d];
  }
}

inline void P3Creator::addNextLevel(Task * nextLevel) {
  nextLevel_.push_back(nextLevel);
}

#endif /* P3CREATOR_H */

