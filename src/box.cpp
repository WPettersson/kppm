/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/


#include "box.h"
#include "boxstore.h"


void Box::split(int * s, BoxStore & store) {
  double * lower = new double[this->dim_];
  double * upper = new double[this->dim_];
  for( int newBoxId = 1; newBoxId < (1 << dim_)-1; ++newBoxId) {
    bool emptyBox = false;
    for( int d = 0; d < dim_; ++d) {
      int o = objectives_[d];
      if ( (newBoxId & (1 << d)) != 0) {
        lower[d] = lower_[d];
        upper[d] = s[o];
        if (lower[d] == upper[d])
          emptyBox = true;
      } else {
        lower[d] = s[o];
        upper[d] = upper_[d];
        if (lower[d] == upper[d])
          emptyBox = true;
      }
    }
    if (! emptyBox) {
      Box *b = new Box(upper, lower, objectives_, dim_);
      store.insert(b);
    }
  }
  delete[] lower;
  delete[] upper;
}
