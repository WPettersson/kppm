/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#ifndef BOXSTORE_H
#define BOXSTORE_H

#include <list>
#include <iostream>

#include "box.h"

class BoxStore {
  public:
    BoxStore(int dim);
    ~BoxStore();
    Box * find(int * sol) const;
    void insert(Box * b);
    void remove(Box * b);
    size_t size() const;

    std::list<Box *>::const_iterator begin() const;
    std::list<Box *>::const_iterator end() const;

  private:
    int dim_;
    std::list<Box *> boxes_;

};

inline BoxStore::BoxStore(int dim) :
    dim_(dim) {
}

inline BoxStore::~BoxStore() {
  for(auto b: boxes_) {
    delete b;
  }
}

inline Box * BoxStore::find(int * sol) const {
  for(auto b: boxes_) {
    if (b->contains(sol))
      return b;
  }
  return nullptr;
}

inline void BoxStore::insert(Box * b) {
  boxes_.push_back(b);
}

inline void BoxStore::remove(Box * b) {
  boxes_.remove(b);
  delete b;
}

inline size_t BoxStore::size() const {
  return boxes_.size();
}

inline std::list<Box *>::const_iterator BoxStore::begin() const {
  return boxes_.begin();
}

inline std::list<Box *>::const_iterator BoxStore::end() const {
  return boxes_.end();
}

#endif /* BOXSTORE_H */
