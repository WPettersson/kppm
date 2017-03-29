/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#include <string>
#include <sstream>

#include "box.h"
#include "boxstore.h"
#include "p3creator.h"
#include "p3task.h"
#include "env.h"
#include "problem.h"
#include "solutions.h"
#include "types.h"

#ifdef DEBUG
extern std::mutex debug_mutex;
#endif


Status P3Creator::operator()() {
  status_ = RUNNING;
  gatherSolutions();
  cleanSolutions();
#ifdef DEBUG
  debug_mutex.lock();
  std::cout << details();
  debug_mutex.unlock();
#endif

  std::vector<Task *> tasks;
  if (solutions().size() > 1) {
#ifdef DEBUG
    debug_mutex.lock();
    std::cout << "Upper: " << upper_[0];
    for(int i = 1; i < objCount_; ++i) {
      std::cout << ", " << upper_[i];
    }
    std::cout << std::endl;
    std::cout << "Lower: " << lower_[0];
    for(int i = 1; i < objCount_; ++i) {
      std::cout << ", " << lower_[i];
    }
    std::cout << std::endl;
    debug_mutex.unlock();
#endif
    BoxStore store(objCount_);
    Solutions * s = nullptr;
    if (shareSolns_) {
      s = new Solutions(objCountTotal_);
    }
    store.insert(new Box(upper_, lower_, objectives_, objCount_));
    for(auto s: solutions()) {
      Box * b = store.find(s);
      if (b == nullptr) {
        std::cerr << "Couldn't find [" << s[0];
        for(int i = 1; i < objCount_; ++i) {
          std::cerr << ", " << s[i];
        }
        std::cerr << "] inside box!!" << std::endl;
      } else {
#ifdef DEBUG
        debug_mutex.lock();
        std::cout << "Sol [" << s[0];
        for(int i = 1; i < objCount_; ++i) {
          std::cout << ", " << s[i];
        }
        std::cout << "]" << std::endl;
        std::cout << "Splitting box " << b->str() << std::endl;
        debug_mutex.unlock();
#endif
        b->split(s, store);
        store.remove(b);
      }
    }
    for(auto b: store) {
#ifdef DEBUG
      debug_mutex.lock();
      std::cout << "P3 task with box " << b->str() << std::endl;
      debug_mutex.unlock();
#endif
      P3Task * p = new P3Task(b, filename_, objCount_, objCountTotal_, objectives_, sense_, s);
      tasks.push_back(p);
    }
  } else {
#ifdef DEBUG
    debug_mutex.lock();
    std::cout << "Upper: " << upper_[0];
    for(int i = 1; i < objCount_; ++i) {
      std::cout << ", " << upper_[i];
    }
    std::cout << std::endl;
    std::cout << "Lower: " << lower_[0];
    for(int i = 1; i < objCount_; ++i) {
      std::cout << ", " << lower_[i];
    }
    std::cout << std::endl;
    debug_mutex.unlock();
#endif
    Box * b = new Box(upper_, lower_, objectives_, objCount_);
    P3Task * p = new P3Task(b, filename_, objCount_, objCountTotal_, objectives_, sense_);
    tasks.push_back(p);
    delete b;
  }

  for (auto n: nextLevel_) {
    for (auto t: tasks) {
      n->addPreReq(t);
    }
  }

  for (auto t: tasks) {
    taskServer_->q(t);
  }

  status_ = DONE;
  return status_;
}

std::string P3Creator::str() const {
  std::stringstream ss;
  ss << "P3Creator: is " << status_ << std::endl;
  return ss.str();
}

// TODO
std::string P3Creator::details() const {
  std::stringstream ss;
  ss << "P3Creator: " << objCount_ << " objectives [";
  for (int i = 0; i < objCount_; ++i) {
    if (i != 0)
      ss << ", ";
    ss << objectives_[i];
  }
  ss << "] with " << solutions().size() << " solutions." << std::endl;
  return ss.str();
}
