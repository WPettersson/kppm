/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#include <cstring>
#include <mutex>

#include "result.h"
#include "solutions.h"

#ifdef DEBUG
extern std::mutex debug_mutex;
#endif

const Result * Solutions::find(const double *ip, const Sense sense) const {
  bool t1,t3;
  for (auto& res: store_) {

    t1 = true;
    t3 = true;
    if (sense == MIN) {
      for (int i = 0; i < objective_count; i++){
        /* t1: All values of candidate must be >= than ip */
        if (res->ip[i] < ip[i]) {
          t1 = false;
          break;
        }
        /* t3: All values of candidate result must be <= than ip */
        if (!res->infeasible) {
          if (res->result[i] > ip[i]) {
            t3 = false;
            break;
          }
        }
      }
    } else {
      for (int i = 0; i < objective_count; i++){
        /* t1: All values of candidate must be <= than ip */
        if (res->ip[i] > ip[i]) {
          t1 = false;
          break;
        }
        /* t3: All values of candidate result must be >= than ip */
        if (!res->infeasible) {
          if (res->result[i] < ip[i]) {
            t3 = false;
            break;
          }
        }
      }
    }
    /* If all conditions are met copy problem & solution and return */
    if (t1 && t3) {
#if defined(DEBUG) && defined(DEBUG_SOLUTION_SEARCH)
      debug_mutex.lock();
      std::cout << " relaxed to ";
      for(int i = 0; i < objective_count; ++i) {
        if (res->ip[i] > 1e19)
          std::cout << "∞,";
        else if (res->ip[i] < -1e19)
          std::cout << "-∞,";
        else
          std::cout << res->ip[i] << ",";
      }
      std::cout << " soln is ";
      if (res->infeasible) {
        std::cout << "infeasible";
      } else {
        for(int i = 0; i < objective_count; ++i) {
          std::cout << res->result[i] << ",";
        }
      }
      std::cout << std::endl;
      debug_mutex.unlock();
#endif
      return res;
    }
  }
#if defined(DEBUG) && defined(DEBUG_SOLUTION_SEARCH)
  debug_mutex.lock();
  std::cout << " no relaxation found" << std::endl;
  debug_mutex.unlock();
#endif
  return nullptr;
}
void Solutions::insert(const double *lp, const int *result,
    const bool infeasible) {
  Result * r = new Result;
  r->objective_count = objective_count;
  r->ip = new double[objective_count];
  for (int i = 0; i < objective_count; ++i) {
    r->ip[i] = lp[i];
  }
  if (infeasible) {
    r->infeasible = true;
    r->result = nullptr;
  } else {
    r->infeasible = false;
    r->result = new int[objective_count];
    for (int i = 0; i < objective_count; ++i) {
      r->result[i] = result[i];
    }
  }
  store_.push_back(r);
}

Solutions::~Solutions() {
  for(auto r: store_) {
    delete[] r->ip;
    if (r->result)
      delete[] r->result;
    delete r;
  }
}
