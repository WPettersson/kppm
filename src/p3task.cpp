/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#include <atomic>
#include <cmath>
#include <string>
#include <sstream>

#include <ilcplex/cplexx.h>

#include "p3task.h"
#include "env.h"
#include "problem.h"
#include "solutions.h"
#include "types.h"

#ifdef DEBUG
extern std::mutex debug_mutex;
#endif

extern std::atomic<int> ipcount;

int P3Task::solve(Env & e, Problem & p, int * result, double * rhs) {

  int cur_numcols, status, solnstat;
  double objval;
  double * srhs;
  srhs = new double[p.objcnt];


  for(int i = 0; i < p.objcnt; ++i)
    srhs[i] = rhs[i];

  cur_numcols = CPXXgetnumcols(e.env, e.lp);
  bool * objectives_done = new bool[p.objcnt];
  for(int i = 0; i < p.objcnt; ++i)
    objectives_done[i] = false;

  for (int pre_j = 0; pre_j < objCount_; pre_j++) {
    int j = objectives_[pre_j];
    objectives_done[j] = true;
    status = CPXXchgobj(e.env, e.lp, cur_numcols, p.objind[j], p.objcoef[j]);
    if (status) {
      std::cerr << "Failed to set objective." << std::endl;
    }

    status = CPXXchgrhs (e.env, e.lp, p.objcnt, p.conind, srhs);
    if (status) {
      std::cerr << "Failed to change constraint srhs" << std::endl;
    }

    /* solve for current objective*/
    status = CPXXmipopt (e.env, e.lp);
    ipcount++;
    if (status) {
      std::cerr << "Failed to optimize LP." << std::endl;
    }

    solnstat = CPXgetstat (e.env, e.lp);
    if ((solnstat == CPXMIP_INFEASIBLE) || (solnstat == CPXMIP_INForUNBD)) {
       break;
    }
    status = CPXXgetobjval (e.env, e.lp, &objval);
    if ( status ) {
      std::cerr << "Failed to obtain objective value." << std::endl;
      exit(0);
    }
    if ( objval > 1/p.mip_tolerance ) {
      while (objval > 1/p.mip_tolerance) {
        p.mip_tolerance /= 10;
      }
      CPXXsetdblparam(e.env, CPXPARAM_MIP_Tolerances_MIPGap, p.mip_tolerance);
      status = CPXmipopt (e.env, e.lp);
      ipcount++;
      solnstat = CPXgetstat (e.env, e.lp);
      if ((solnstat == CPXMIP_INFEASIBLE) || (solnstat == CPXMIP_INForUNBD)) {
        break;
      }
      status = CPXXgetobjval (e.env, e.lp, &objval);
      if ( status ) {
        std::cerr << "Failed to obtain objective value." << std::endl;
        exit(0);
      }
    }
    result[j] = srhs[j] = round(objval);
  }

  // Get the solution vector
  double * soln = new double[cur_numcols];
  CPXgetx(e.env, e.lp, soln, 0, cur_numcols - 1);
  // Now run through the rest of the objectives.
  for (int j = 0; j < p.objcnt; j++) {
    if (objectives_done[j])
      continue;
    double res = 0;
    for(int i = 0; i < cur_numcols; ++i) {
      res += p.objcoef[j][i] * soln[i];
    }
    result[j] = round(res);
  }

  delete[] srhs;

  return solnstat;
}


Status P3Task::operator()() {
  status_ = RUNNING;
#ifdef DEBUG
  std::cout << details();
#endif
  Env e;

  int status;
  /* Initialize the CPLEX environment */
  e.env = CPXopenCPLEX (&status);
  if (status != 0) {
    std::cerr << "Could not open CPLEX environment." << std::endl;
  }

  /* Set to deterministic parallel mode */
  status = CPXsetintparam(e.env, CPXPARAM_Parallel, CPX_PARALLEL_DETERMINISTIC);

  /* Set to only one thread */
  CPXsetintparam(e.env, CPXPARAM_Threads, 1);

  if (e.env == NULL) {
    std::cerr << "Could not open CPLEX environment." << std::endl;
  }

  status = CPXsetintparam(e.env, CPX_PARAM_SCRIND, CPX_OFF);
  if (status) {
    std::cerr << "Failure to turn off screen indicator." << std::endl;
  }

  Problem p(filename_.c_str(), e);

#ifdef FINETIMING
  double cplex_time = 0;
  double wait_time = 0;
  timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);
  double total_time = start.tv_sec + start.tv_nsec/1e9;
#endif

  Solutions s(p.objcnt);
  int infcnt;
  bool inflast;
  bool infeasible;
  int * max, * min,  * result, *resultStore;
  double * rhs;

  result = resultStore = new int[p.objcnt];
  rhs = new double[p.objcnt];
  for(int i = 0; i < p.objcnt; ++i) {
    rhs[i] = p.rhs[i];
  }

  if (p.objsen == MIN) {
    for (int i = 1; i < objCount_; ++i) {
      double start = bounds_[1][i];
      rhs[objectives_[i]] = start;
    }
  } else {
    for (int i = 1; i < objCount_; ++i) {
      double start = bounds_[0][i];
      rhs[objectives_[i]] = start;
    }
  }

#ifdef FINETIMING
  clock_gettime(CLOCK_MONOTONIC, &start);
  double starttime = (start.tv_sec + start.tv_nsec/1e9);
#endif
  int solnstat = solve(e, p, result, rhs);
#ifdef FINETIMING
  clock_gettime(CLOCK_MONOTONIC, &start);
  cplex_time += (start.tv_sec + start.tv_nsec/1e9) - starttime;
#endif
#ifdef DEBUG
  debug_mutex.lock();
  std::cout << *this << " with constraints ";
  for(int i = 0; i < p.objcnt; ++i) {
    if (rhs[i] > 1e09)
      std::cout << "∞";
    else if (rhs[i] < -1e09)
      std::cout << "-∞";
    else
      std::cout << rhs[i];
    std::cout << ",";
  }
  std::cout << " found ";
  if (solnstat == CPXMIP_INFEASIBLE) {
    std::cout << "infeasible";
  } else {
    for(int i = 0; i < p.objcnt; ++i) {
      std::cout << result[i] << ",";
    }
  }
  std::cout << std::endl;
  debug_mutex.unlock();
#endif
  Sense sense = p.objsen;
  /* Need to add a result to the list here*/
  s.insert(rhs, result, solnstat == CPXMIP_INFEASIBLE);
  // Note that if we are splitting, we aren't sharing.
  min = new int[objCountTotal_];
  max = new int[objCountTotal_];


  if (solnstat != CPXMIP_INFEASIBLE) {
    for (int j = 0; j < objCountTotal_; j++) {
      min[j] = max[j] = result[j];
    }
  }
  for (int objective_counter = 1; objective_counter < objCount_; objective_counter++) {
    int objective = objectives_[objective_counter];
    int depth_level = 1; /* Track current "recursion" depth */
    int depth = objectives_[depth_level]; /* Track current "recursion" depth */
    bool onwalk = false; /* Are we on the move? */
    infcnt = 0; /* Infeasible count*/
    inflast = false; /* Last iteration infeasible?*/


    /* Set all constraints back to infinity*/
    for (int j = 1; j < objCount_; ++j) {
      if (sense == MIN) {
        rhs[j] = CPX_INFBOUND;
      } else {
        rhs[j] = -CPX_INFBOUND;
      }
    }
    if (sense == MIN) {
      for (int i = 1; i < objCount_; ++i) {
        double start = bounds_[1][i];
        rhs[objectives_[i]] = start;
      }
    } else {
      for (int i = 1; i < objCount_; ++i) {
        double start = bounds_[0][i];
        rhs[objectives_[i]] = start;
      }
    }
    /* Set rhs of current depth */
    if (sense == MIN) {
      rhs[objective] = max[objective]-1;
    } else {
      rhs[objective] = min[objective]+1;
    }

    max[objective] = (int) -CPX_INFBOUND;
    min[objective] = (int) CPX_INFBOUND;

    while (infcnt < objective_counter) {
      bool relaxed;
      int solnstat;
      /* Look for possible relaxations to the current problem*/
      const Result *relaxation;

#ifdef DEBUG_SOLUTION_SEARCH
      debug_mutex.lock();
      std::cout << "Thread " << *this;
      debug_mutex.unlock();
#endif
      // First check if it's infeasible
      relaxation = s.find(rhs, p.objsen);
      relaxed = (relaxation != nullptr);
      if (relaxed) {
        infeasible = relaxation->infeasible;
        result = relaxation->result;
      } else {
        /* Solve in the absence of a relaxation*/
        result = resultStore;
#ifdef FINETIMING
        clock_gettime(CLOCK_MONOTONIC, &start);
        double starttime = (start.tv_sec + start.tv_nsec/1e9);
#endif
        solnstat = solve(e, p, result, rhs);
#ifdef FINETIMING
        clock_gettime(CLOCK_MONOTONIC, &start);
        cplex_time += (start.tv_sec + start.tv_nsec/1e9) - starttime;
#endif
        infeasible = ((solnstat == CPXMIP_INFEASIBLE) || (solnstat == CPXMIP_INForUNBD));
        /* Store result */
        s.insert(rhs, result, infeasible);
      }
#ifdef DEBUG
      debug_mutex.lock();
      std::cout << *this << " with constraints ";
      for(int i = 0; i < p.objcnt; ++i) {
        if (rhs[i] > 1e09)
          std::cout << "∞";
        else if (rhs[i] < -1e09)
          std::cout << "-∞";
        else
          std::cout << rhs[i];
        std::cout << ",";
      }
      std::cout << " found ";
      if (relaxed)
        std::cout << "relaxation ";
      if (infeasible) {
        std::cout << "infeasible." << std::endl;
      } else {
        for(int i = 0; i < p.objcnt; ++i) {
          std::cout << result[i] << ",";
        }
        std::cout << std::endl;
      }
      debug_mutex.unlock();
#endif
      // See if we've gone past our boundary
      if (!infeasible) {
        if (sense == MIN) {
          for (int i = 1; i < objCount_; ++i) {
            double stop = bounds_[0][i];
            if (rhs[objectives_[i]] < stop) {
              infeasible = true;
              break;
            }
          }
        } else {
          for (int i = 1; i < objCount_; ++i) {
            double stop = bounds_[1][i];
            if (rhs[objectives_[i]] > stop) {
              infeasible = true;
              break;
            }
          }
        }
      }
      if (infeasible) {
        infcnt++;
        inflast = true;
      } else {
        infcnt = 0;
        inflast = false;
        /* Update maxima */
        for (int j = 0; j < objCountTotal_; j++) {
          if (result[j] > max[j]) {
            max[j] = result[j];
          }
        }
        /* Update minima */
        for (int j = 0; j < objCountTotal_; j++) {
          if (result[j] < min[j]) {
            min[j] = result[j];
          }
        }
      }

      if (infeasible && (infcnt == objective_counter-1)) {
        /* Set all constraints back to infinity */
        for (int j = 0; j < objCountTotal_; j++) {
          if (sense == MIN) {
            rhs[j] = CPX_INFBOUND;
          } else {
            rhs[j] = -CPX_INFBOUND;
          }
        }
        // Reset to start point, not to infinity, if we know the start point!
        for(int i = 1; i < objCount_; ++i) {
          if (sense == MIN) {
            rhs[objectives_[i]] = bounds_[1][i];
          } else {
            rhs[objectives_[i]] = bounds_[0][i];
          }
        }
        /* In the case of a minimisation problem
         * set current level to max objective function value  -1 else set
         * current level to min objective function value  +1 */
        if (sense == MIN) {
          if (max[objective] > (int) -CPX_INFBOUND) {
            double start = INF;
            for(int i = 0; i < objCount_; ++i) {
              if (objectives_[i] == objective) {
                start = bounds_[1][i];
              }
            }
            if (start < max[objective]-1) {
              rhs[objective] = start;
            } else {
              rhs[objective] = max[objective]-1;
            }
            max[objective] = (int) -CPX_INFBOUND;
          }
        } else {
          if (min[objective] < (int) CPX_INFBOUND) {
            double start = -INF;
            for(int i = 0; i < objCount_; ++i) {
              if (objectives_[i] == objective) {
                start = bounds_[0][i];
              }
            }
            if (start > min[objective]+1) {
              rhs[objective] = start;
            } else {
              rhs[objective] = min[objective]+1;
            }
            min[objective] = (int) CPX_INFBOUND;
          }
        }

        /* Reset depth */
        depth_level = 1;
        depth = objectives_[depth_level];
        onwalk = false;
      } else if (inflast && infcnt != objective_counter) {
        if (sense == MIN) {
          rhs[depth] = CPX_INFBOUND;
        } else {
          rhs[depth] = -CPX_INFBOUND;
        }
        // Reset to start point, not to infinity, if we know the start point!
        for(int i = 1; i < objCount_; ++i) {
          if (objectives_[i] == depth) {
            if (sense == MIN) {
              rhs[depth] = bounds_[1][i];
            } else {
              rhs[depth] = bounds_[0][i];
            }
          }
        }
        depth_level++;
        depth = objectives_[depth_level];
        if (sense == MIN) {
          rhs[depth] = max[depth]-1;
          max[depth] = (int) -CPX_INFBOUND;
        } else {
          rhs[depth] = min[depth]+1;
          min[depth] = (int) CPX_INFBOUND;
        }
        onwalk = true;
      } else if (!onwalk && infcnt != 1) {
        if (sense == MIN) {
          rhs[depth] = max[depth]-1;
          max[depth] = (int) -CPX_INFBOUND;
        } else {
          rhs[depth] = min[depth]+1;
          min[depth] = (int) CPX_INFBOUND;
        }
      } else if (onwalk && infcnt != 1)  {
        depth_level = 1;
        depth = objectives_[depth_level];
        if (sense == MIN) {
          rhs[depth] = max[depth]-1;
          max[depth] = (int) -CPX_INFBOUND;
        } else {
          rhs[depth] = min[depth]+1;
          min[depth] = (int) CPX_INFBOUND;
        }
        onwalk = false;
      }
    }
  }
#ifdef FINETIMING
  clock_gettime(CLOCK_MONOTONIC, &start);
  total_time = start.tv_sec + start.tv_nsec/1e9 - total_time;
  std::cout << "Thread " << t->id << " used " << cplex_time << "s in cplex";
  std::cout << ", waited for " << wait_time << "s";
  std::cout << " and " << total_time << "s overall." << std::endl;
#endif
  delete[] resultStore;
  delete[] rhs;
  delete[] min;
  delete[] max;

  for(Result * r: s) {
    if (r->infeasible)
      continue;
    int * n = new int[p.objcnt];
    for (int i = 0; i < p.objcnt; ++i) {
      n[i] = r->result[i];
    }
    solutions_.push_back(n);
  }
  status_ = DONE;
  return status_;
}

std::string P3Task::str() const {
  std::stringstream ss;
  ss << "P3Task: " << objCount_ << " objectives [";
  for (int i = 0; i < objCount_; ++i) {
    if (i != 0)
      ss << ", ";
    ss << objectives_[i];
  }
  ss << "]";
  return ss.str();
}

// TODO
std::string P3Task::details() const {
  std::stringstream ss;
  ss << "P3Task: is " << status_ << std::endl;
  ss << "[O" << objectives_[0] << " free]" << std::endl;
  for (int i = 1; i < objCount_ ; ++i) {
    ss << "[";
    ss << bounds_[0][i] << " ≤ O" << objectives_[i] << " ≤ " << bounds_[1][i];
    ss << "]" << std::endl;;
  }
  return ss.str();
}
