#ifndef P2TASK_H
#define P2TASK_H

#ifdef DEBUG
#include <mutex>
#endif

#include "task.h"
#include "env.h"
#include "problem.h"

class P2Task : public Task {
  public:
    P2Task(double **bound, std::string & filename, int objCount,
        int objCountTotal, int * objectives);
    Status operator()();


    virtual std::string str() const;
    virtual std::string details() const;
  private:
    int solve(Env & e, Problem & p, int * result, double * rhs);
    double **bounds_;
#ifdef DEBUG
    std::mutex debug_mutex;
#endif
};

inline P2Task::P2Task(double **bound, std::string & filename, int objCount,
    int objCountTotal, int * objectives) :
    Task(filename, objCount, objCountTotal, objectives) {
  bounds_ = new double*[2];
  bounds_[0] = new double[objCount_];
  bounds_[1] = new double[objCount_];
  for (int d = 0; d < objCount_; ++d) {
    bounds_[0][d] = bound[0][d];
    bounds_[1][d] = bound[1][d];
  }
}

#endif /* P2TASK_H */

