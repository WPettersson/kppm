/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#include <iostream>
#include <list>
#include <cmath>
#include <string>
#include <sstream>

#include "types.h"
#include "p1task.h"
#include "p2task.h"
#include "p3creator.h"

#ifdef DEBUG
extern std::mutex debug_mutex;
#endif


Status P1Task::operator()() {
  status_ = RUNNING;
#ifdef DEBUG
  debug_mutex.lock();
  std::cout << "Running " << *this << std::endl;
  debug_mutex.unlock();
#endif
  int dim = objCount_ - 1;
  int numBlocks = pow(numSteps_,dim);
  std::list<P2Task *> tasks;
  gatherSolutions();
  if (objCount_ == 1) {
    double ** bounds = new double*[2];
    bounds[0] = new double[1];
    bounds[1] = new double[1];
    bounds[0][0] = -INF;
    bounds[1][0] = INF;
    P2Task * p = new P2Task(bounds, filename_, objCount_, objCountTotal_, objectives_, sense_);
    for(auto n: nextLevel_) {
      n->addPreReq(p);
    }
    taskServer_->q(p);
  } else {
    // First calculate absolute max/min values
    double * maxOverall = new double[objCount_];
    double * minOverall = new double[objCount_];
    for (int i = 0; i < objCount_; ++i) {
      maxOverall[i] = -INF;
      minOverall[i] = INF;
    }
    for (int i = 0; i < objCount_; ++i) {
      int o = objectives_[i];
      for (int *s : solutions()) {
        if (s[o] > maxOverall[i])
          maxOverall[i] = s[o];
        if (s[o] < minOverall[i])
          minOverall[i] = s[o];
      }
    }
#ifdef DEBUG
    debug_mutex.lock();
    std::cout << "Upper: [" << maxOverall[0];
    for(int i = 1; i < objCount_; ++i) {
      std::cout << ", " << maxOverall[i];
    }
    std::cout << "]" << std::endl;
    std::cout << "Lower: [" << minOverall[0];
    for(int i = 1; i < objCount_; ++i) {
      std::cout << ", " << minOverall[i];
    }
    std::cout << "]" << std::endl;
    debug_mutex.unlock();
#endif
    for (int b = 0; b < numBlocks; ++b) {
      int temp = b;
      double ** bounds = new double*[2];
      bounds[0] = new double[objCount_];
      bounds[1] = new double[objCount_];
      for (int d = 1; d < objCount_; ++d) {
        int o = objectives_[d];
        double max = maxOverall[d];
        double min = minOverall[d];
        // Find max and min values reached that also satisfy this particular
        // bound so far
        for ( int *s : solutions()) {
          bool valid = true;
          // For all dimensions up to this one
          for (int d_ = 1; d_ < d; ++d_) {
            int o_ = objectives_[d_];

            // Note that we can only check "upper" bounds for minimisation
            // problems (and lower bounds for maximisation problems) on a
            // per-bounds basis. This is what the k-PPM paper says, and 4AP05,
            // with 3 splits per objective, misses results if we try to check
            // both upper and lower bounds on a per-bound basis.

            // Remember, solutions (s) give values for all objectives in their
            // correct order, while bounds[0][d] is the bound on objective[d].
            if ((sense_ == MIN) && (s[o_] > bounds[1][d_]) ) {
              valid = false;
              break;
            }
            if ((sense_ == MAX) && (s[o_] < bounds[0][d_]) ) {
              valid = false;
              break;
            }
          }
          if (valid) {
            if (s[o] > max)
              max = s[o];
            if (s[o] < min)
              min = s[o];
          }
        }
        double stepSize = (max - min) / (numSteps_);
        bounds[0][d] = min + (temp % numSteps_)*stepSize;
        bounds[1][d] = min + (temp % numSteps_ + 1)*stepSize;
        temp = temp / numSteps_;
      }
      P2Task * p = new P2Task(bounds, filename_, objCount_, objCountTotal_, objectives_, sense_);
      tasks.push_back(p);
    }
    P3Creator * p3c = new P3Creator(filename_, objCount_, objCountTotal_,
        objectives_, sense_, minOverall, maxOverall, taskServer_);
    for (auto n: nextLevel_) {
      n->addPreReq(p3c);
      p3c->addNextLevel(n);
    }

    for(auto t: tasks) {
      p3c->addPreReq(t);
      taskServer_->q(t);
    }
    taskServer_->q(p3c);
  }
  status_ = DONE;
#ifdef DEBUG
  debug_mutex.lock();
  std::cout << *this << " done." << std::endl;
  debug_mutex.unlock();
#endif
  return status_;
}

std::string P1Task::str() const {
  std::stringstream ss;
  ss << "P1Task: " << objCount_ << " objectives [";
  for (int i = 0; i < objCount_; ++i) {
    if (i != 0)
      ss << ", ";
    ss << objectives_[i];
  }
  ss << "]";
  return ss.str();
}

std::string P1Task::details() const {
  std::stringstream ss(str());
  ss << std::endl << "P1Task " << this << " is " << status_ << std::endl;
  return ss.str();
}
