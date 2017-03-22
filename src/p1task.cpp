#include <iostream>
#include <list>
#include <cmath>
#include <string>
#include <sstream>

#include "types.h"
#include "p1task.h"
#include "p2task.h"



Status P1Task::operator()() {
  status_ = RUNNING;
#ifdef DEBUG
  std::cout << "Running " << *this << std::endl;
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
    P2Task * p = new P2Task(bounds, filename_, objCount_, objCountTotal_, objectives_);
    tasks.push_back(p);
  } else {
    for (int b = 0; b < numBlocks; ++b) {
      int temp = b;
      double ** bounds = new double*[2];
      bounds[0] = new double[objCount_];
      bounds[1] = new double[objCount_];
      for (int d = 1; d < objCount_; ++d) {
        int o = objectives_[d];
        double max = -INF;
        double min = INF;
        // Find max and min values reached that also satisfy this particular
        // bound so far
        for ( int *s : solutions()) {
          bool valid = true;
          // For all dimensions up to this one
          for (int d_ = 1; d_ < d; ++d_) {
            int o_ = objectives_[d_];
            // Remember, solutions (s) give values for all objectives in their
            // correct order, while bounds[0][d] is the bound on objective[d].
            if ( (s[o_] < bounds[0][d_]) || (s[o_] > bounds[1][d_]) ) {
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
      P2Task * p = new P2Task(bounds, filename_, objCount_, objCountTotal_, objectives_);
      tasks.push_back(p);
    }
  }
  for (auto n: nextLevel_) {
    for(auto t: tasks) {
      n->addPreReq(t);
    }
  }
  for(auto t: tasks) {
    taskServer_->q(t);
  }
  status_ = DONE;
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
  ss << std::endl << "Task is " << status_ << std::endl;
  return ss.str();
}
