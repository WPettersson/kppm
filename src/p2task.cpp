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

#include "p2task.h"
#include "env.h"
#include "problem.h"
#include "solutions.h"
#include "types.h"

extern std::atomic<int> ipcount;

#ifdef DEBUG
extern std::mutex debug_mutex;
#endif


Status P2Task::operator()() {
  status_ = RUNNING;
#ifdef DEBUG
  debug_mutex.lock();
  std::cout << details();
  debug_mutex.unlock();
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
  int cur_numcols = CPXgetnumcols(e.env, e.lp);
  // Add our constraints
  for(int d = 1 ; d < objCount_; ++d) {
    int o = objectives_[d];
    char sense[2] = "G";
    CPXDIM ccnt = 0; // column added count
    CPXDIM rcnt = 1; // row added count
    CPXNNZ nzcnt = cur_numcols; // non-zero entries
    CPXDIM * rmatind = new CPXDIM[cur_numcols];
    for(int i = 0; i < cur_numcols; ++i) {
      rmatind[i] = i;
    }
    CPXNNZ rmatbeg[1] { 0 };
    const char* rowName[1];
    std::stringstream rowString;
    rowString << "Lower bound on " << o;
    std::string str = rowString.str();
    rowName[0] = str.c_str();
    status = CPXXaddrows(e.env, e.lp, ccnt, rcnt, nzcnt,
        &bounds_[0][d], // rhs
        sense, rmatbeg, rmatind,
        p.objcoef[o], // rmatval
        NULL, // colname
        NULL);

    sense[0] = 'L';
    rowString.str(std::string());
    rowString << "Upper bound on " << o;
    str = rowString.str();
    rowName[0] = str.c_str();
    status = CPXXaddrows(e.env, e.lp, ccnt, rcnt, nzcnt,
        &bounds_[1][d], // rhs
        sense, rmatbeg, rmatind,
        p.objcoef[o], // rmatval
        NULL, // colname
        NULL);
  }

#ifdef FINETIMING
  double cplex_time = 0;
  double wait_time = 0;
  timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);
  double total_time = start.tv_sec + start.tv_nsec/1e9;
#endif

  int solnstat;
  double * sol = new double[objCountTotal_];
  int o = objectives_[obj_];
  status = CPXXchgobj(e.env, e.lp, cur_numcols, p.objind[o], p.objcoef[o]);
  if (status) {
    std::cerr << "Failed to set objective." << std::endl;
  }


  /* solve for current objective*/
  status = CPXXmipopt (e.env, e.lp);
  ipcount++;
  if (status) {
    std::cerr << "Failed to optimize LP." << std::endl;
  }

  solnstat = CPXgetstat (e.env, e.lp);
  if ((solnstat == CPXMIP_INFEASIBLE) || (solnstat == CPXMIP_INForUNBD)) {
    status_ = DONE;
    return status_;
  }
  status = CPXXgetobjval (e.env, e.lp, &sol[o]);
  if ( status ) {
    std::cerr << "Failed to obtain objective value." << std::endl;
    exit(0);
  }
  if ( sol[o]> 1/p.mip_tolerance ) {
    while (sol[o] > 1/p.mip_tolerance) {
      p.mip_tolerance /= 10;
    }
    CPXXsetdblparam(e.env, CPXPARAM_MIP_Tolerances_MIPGap, p.mip_tolerance);
    status = CPXmipopt (e.env, e.lp);
    ipcount++;
    solnstat = CPXgetstat (e.env, e.lp);
    if ((solnstat == CPXMIP_INFEASIBLE) || (solnstat == CPXMIP_INForUNBD)) {
      return status_ = DONE;
    }
    status = CPXXgetobjval (e.env, e.lp, &sol[o]);
    if ( status ) {
      std::cerr << "Failed to obtain objective value." << std::endl;
      exit(0);
    }
  }

  // Get the solution vector
  double * soln = new double[cur_numcols];
  CPXgetx(e.env, e.lp, soln, 0, cur_numcols - 1);
  // Now run through the rest of the objectives.
  for (int j = 0; j < objCountTotal_; j++) {
    if (j == o)
      continue;
    double res = 0;
    for(int i = 0; i < cur_numcols; ++i) {
      res += p.objcoef[j][i] * soln[i];
    }
    sol[j] = round(res);
  }

  int * n = new int[objCountTotal_];
  for (int i = 0; i < objCountTotal_; ++i) {
    n[i] = round(sol[i]);
  }
  solutions_.push_back(n);

#ifdef DEBUG
  debug_mutex.lock();
  std::cout << "P2Task: found [";
  for (int i = 0; i < objCountTotal_; ++i) {
    if (i != 0)
      std::cout << ", ";
    std::cout << sol[i];
  }
  std::cout << "]";
  std::cout << std::endl;
  debug_mutex.unlock();
#endif
  status_ = DONE;
  return status_;
}

std::string P2Task::str() const {
  std::stringstream ss;
  ss << "P2Task: " << objCount_ << " objectives [";
  for (int i = 0; i < objCount_; ++i) {
    if (i != 0)
      ss << ", ";
    ss << objectives_[i];
  }
  ss << "]";
  return ss.str();
}

// TODO
std::string P2Task::details() const {
  std::stringstream ss;
  ss << "P2Task: is " << status_ << std::endl;
  ss << "[O" << objectives_[0] << " free]" << std::endl;
  for (int i = 1; i < objCount_ ; ++i) {
    ss << "[";
    ss << bounds_[0][i] << " ≤ O" << objectives_[i] << " ≤ " << bounds_[1][i];
    ss << "]" << std::endl;;
  }
  return ss.str();
}
