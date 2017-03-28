/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef P3TASK_H
#define P3TASK_H

#ifdef DEBUG
#include <mutex>
#endif

#include "box.h"
#include "task.h"
#include "env.h"
#include "problem.h"

class P3Task : public Task {
  public:
    P3Task(Box * b, std::string & filename, int objCount,
        int objCountTotal, int * objectives, Sense sense);
    Status operator()();


    virtual std::string str() const;
    virtual std::string details() const;
  private:
    int solve(Env & e, Problem & p, int * result, double * rhs);
    double **bounds_;
};

inline P3Task::P3Task(Box * b, std::string & filename, int objCount,
    int objCountTotal, int * objectives, Sense sense) :
    Task(filename, objCount, objCountTotal, objectives, sense) {
  bounds_ = new double*[2];
  bounds_[0] = new double[objCount_];
  bounds_[1] = new double[objCount_];
  for (int d = 0; d < objCount_; ++d) {
    bounds_[0][d] = b->lower(d);
    bounds_[1][d] = b->upper(d);
  }
}

#endif /* P3TASK_H */

