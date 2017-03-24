/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef GATHER_H
#define GATHER_H

#include <sstream>

#include "task.h"

class Gather : public Task {
  public:
    Gather(std::string problem, int objCount, int * objectives, Sense sense);

    virtual Status operator()();

    virtual std::string str() const;
    virtual std::string details() const;

};

inline Gather::Gather(std::string problem, int objCount,
    int * objectives, Sense sense) :
    Task(problem, objCount, objCount, objectives, sense) {
}

inline Status Gather::operator()() {
  gatherSolutions();
  return status_;
}

inline std::string Gather::str() const {
  return std::string("Gathering task");
}

inline std::string Gather::details() const {
  std::stringstream ss(str());
  ss << " is " << status_ << std::endl;
  return ss.str();
}

#endif /* GATHER_H */
