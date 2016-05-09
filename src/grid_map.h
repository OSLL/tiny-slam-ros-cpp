/*
Copyright (c) 2016 JetBrains Research, Mobile Robot Algorithms Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy of this 
software and associated documentation files (the "Software"), to deal in the Software 
without restriction, including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>

#include "geometry_utils.h"

template<typename Cell>
class GridMap {
public:
  GridMap(): // TODO: replace hardcoded value with params
    _width(250), _height(250), _m_per_cell(0.2),
    _cells(_height) {
    for (auto &row : _cells) {
      row.resize(_width);
    }
  }

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  void update_cell(const DiscretePoint2D& cell_coord,
                   double new_value, double quality = 1.0) {
    // TODO: bounds check
    _cells[cell_coord.y][cell_coord.x].set_value(new_value, quality);
  }

  double cell_value(const DiscretePoint2D& cell_coord) const {
    return _cells[cell_coord.y][cell_coord.x].value();
  }

  const Cell &cell(const DiscretePoint2D& cell_coord) const {
    return _cells[cell_coord.y][cell_coord.x];
  }

  const Cell &cell(int x, int y) const {
      return _cells[x][y];
  }

  Cell &cell(const DiscretePoint2D& cell_coord) {
    return _cells[cell_coord.y][cell_coord.x];
  }

  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.y < _width &&
           0 <= cell_coord.y && cell_coord.y < _height;
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_y = _height/2 + std::round(y/_m_per_cell);
    int cell_x = _width /2 + std::round(x/_m_per_cell);

    return DiscretePoint2D(cell_x, cell_y);
  }

  double cell_scale() const { return _m_per_cell; }

private: // fields
  int _width, _height;
  double _m_per_cell;
  std::vector<std::vector<Cell>> _cells;
};

#endif
