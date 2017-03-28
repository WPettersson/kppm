/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef TASK_H
#define TASK_H

#include <algorithm>
#include <list>
#include <mutex>
#include <string>
#include <iostream>

#include "sense.h"

#ifdef DEBUG
extern std::mutex debug_mutex;
#endif

/**
 * Status of a task:
 * WAITING - waiting for pre-requisites to complete
 * QUEUED - pre-requisites complete, waiting to start
 * RUNNING - running
 * DONE - done
 */
enum Status { WAITING, QUEUED, RUNNING, DONE };

class Task {
  public:
    Task(std::string filename, int objCount, int objCountTotal,
        int * objectives, Sense sense);
    ~Task();

    bool isReady() const;
    Status status() const;
    void dumpSolutions(std::ostream & out) const;
    void cleanSolutions();
    void removeNonPareto();
    int objCount() const;
    int objective(int i) const;

    void addPreReq(Task *t);

    virtual Status operator()() = 0;

    const std::list<int *> solutions() const;

    virtual std::string str() const = 0;
    virtual std::string details() const = 0;

  protected:
    void gatherSolutions();
    void sortSolutions();
    void removeDuplicates();
    Status status_;
    std::mutex listMutex_;
    std::list<Task *> preReqs_;

    std::list<int*> solutions_;
    std::string filename_;
    int objCount_;
    int objCountTotal_;
    int * objectives_;
    Sense sense_;

};

std::ostream & operator<<(std::ostream & str, const Task & t);

inline Task::Task(std::string filename, int objCount, int objCountTotal,
    int * objectives, Sense sense) : filename_(filename),
    objCount_(objCount), objCountTotal_(objCountTotal), sense_(sense) {
  objectives_ = new int[objCount_];
  status_ = WAITING;
  for (int i = 0; i < objCount_; ++i) {
    objectives_[i] = objectives[i];
  }
}

inline Task::~Task() {
  delete[] objectives_;
}

inline void Task::addPreReq(Task * t) {
  std::unique_lock<std::mutex> lock(listMutex_);
  preReqs_.push_back(t);
}

inline Status Task::status() const {
  return status_;
}

inline bool Task::isReady() const {
#ifdef DEBUG_TASKSERVER
  debug_mutex.lock();
  std::cout << "Checking if " << *this << " is ready" << std::endl;
  debug_mutex.unlock();
#endif
  for(auto task: preReqs_) {
    if (task->status() != DONE) {
#ifdef DEBUG_TASKSERVER
      debug_mutex.lock();
      std::cout << "Still waiting on " << *task << std::endl;
      debug_mutex.unlock();
#endif
      return false;
    }
  }
#ifdef DEBUG_TASKSERVER
  debug_mutex.lock();
  std::cout << *this << " is ready" << std::endl;
  debug_mutex.unlock();
#endif
  return true;
}

inline int Task::objCount() const {
  return objCount_;
}

inline int Task::objective(int i) const {
  return objectives_[i];
}

inline void Task::gatherSolutions() {
  for(auto t: preReqs_) {
    for(int * s: t->solutions()) {
      int * n = new int[objCountTotal_];
      for(int i = 0; i < objCountTotal_; ++i) {
        n[i] = s[i];
      }
      solutions_.push_back(n);
    }
  }
}

inline void Task::cleanSolutions() {
  sortSolutions();
  removeDuplicates();
  removeNonPareto();
}

inline void Task::sortSolutions() {
//  std::sort(solutions_.begin(), solutions_.end(),
  solutions_.sort(
      [this](int * a, int * b) {
        for(int i = 0; i < this->objCount_; ++i) {
          if (a[i] < b[i])
            return false;
          if (a[i] > b[i])
            return true;
        }
        return true;
      });
}

inline void Task::removeDuplicates() {
  solutions_.erase( std::unique(solutions_.begin(), solutions_.end(),
        [this](int * a, int * b) {
        for(int i = 0; i < this->objCount_; ++i) {
          if (a[i] != b[i])
            return false;
        }
        return true;
      }), solutions_.end());
}

inline void Task::removeNonPareto() {
  solutions_.erase( std::remove_if(solutions_.begin(), solutions_.end(),
        [this](int * a) {
        for(int * b: solutions_) {
          bool notWorse = true;
          bool better = false;
          for(int i = 0; i < this->objCount_ && notWorse; ++i) {
            if ((sense_ == MIN) && (a[i] < b[i]))
              notWorse = false;
            if ((sense_ == MIN) && (a[i] > b[i]))
              better = true;
            if ((sense_ == MAX) && (a[i] > b[i]))
              notWorse = false;
            if ((sense_ == MAX) && (a[i] < b[i]))
              better = true;
          }
          if (notWorse && better) {
            return true;
          }
        }
        return false;
      }), solutions_.end());
}

inline void Task::dumpSolutions(std::ostream & out) const {
  for (int * s: solutions()) {
    out << s[0];
    for (int i  = 1; i < objCountTotal_; ++i) {
      out << "\t" << s[i];
    }
    out << std::endl;
  }
}

const inline std::list<int *> Task::solutions() const {
  return solutions_;
}

inline std::ostream & operator<<(std::ostream & str, Status status_) {
  std::string res = "UNKNOWN STATE!";
  switch (status_) {
    case WAITING:
      res = "WAITING";
      break;
    case QUEUED:
      res = "QUEUED";
      break;
    case RUNNING:
      res = "RUNNING";
      break;
    case DONE:
      res = "DONE";
      break;
    default:
      break;
  }
  return str << res;
}

inline std::ostream & operator<<(std::ostream & str, const Task & t) {
  return str << t.str();
}

#endif /* TASK_H */
