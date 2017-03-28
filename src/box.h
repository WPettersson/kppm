/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef BOX_H
#define BOX_H

#include <string>
#include <sstream>

class BoxStore;

class Box {
  public:
    Box(double * upper, double * lower, int * objectives, int dim);
    ~Box();
    bool contains(int * s);

    void split(int * s, BoxStore & b);
    double lower(int i) const;
    double upper(int i) const;

    std::string str() const;
  private:
    double * upper_;
    double * lower_;
    int * objectives_;
    int dim_;
};

inline Box::Box(double * upper, double * lower, int * objectives, int dim): dim_(dim) {
  upper_ = new double[dim_];
  lower_ = new double[dim_];
  objectives_ = new int[dim_];
  for(int i = 0; i < dim_; ++i) {
    objectives_[i] = objectives[i];
    upper_[i] = upper[i];
    lower_[i] = lower[i];
  }
}

inline Box::~Box() {
  delete[] upper_;
  delete[] lower_;
  delete[] objectives_;
}

inline bool Box::contains(int * s) {
  for(int i = 0; i < dim_; ++i) {
    int o = objectives_[i];
    if (s[o] < lower_[i])
      return false;
    if (s[o] > upper_[i])
      return false;
  }
  return true;
}

inline double Box::upper(int i) const {
  return upper_[i];
}

inline double Box::lower(int i) const {
  return lower_[i];
}

inline std::string Box::str() const {
  std::stringstream ss;
  ss << "Box: [" << lower_[0];
  for(int i = 1; i < dim_; ++i) {
    ss << ", " << lower_[i];
  }
  ss << "] => [" << upper_[0];
  for(int i = 1; i < dim_; ++i) {
    ss << ", " << upper_[i];
  }
  ss << "]";
  return ss.str();
}


#endif /* BOX_H */
