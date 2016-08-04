#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>
#include <iostream>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

class GridMap {
private: //flag constants
  const int RESIZE_DOWN  = 1; // 0b0001
  const int RESIZE_UP    = 2; // 0b0010
  const int RESIZE_LEFT  = 4; // 0b0100
  const int RESIZE_RIGHT = 8; // 0b1000
  const int RESIZE_VERT  = 3; // 0b0011
  const int RESIZE_HORZ  = 12;// 0b1100
  const int RESIZE_NEED  = 15;// 0b1111
public: // typedefs
  using Cell = std::shared_ptr<GridCell>;
private: // typedefs
  using RowContainer = std::vector<Cell>;
  using GridMapContainer = std::vector<RowContainer>;

public:
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCellFactory> cell_factory):
    // TODO: replace hardcoded value with params
    _width(1), _height(1), _m_per_cell(0.1),
    // the map size 1x1 is chosen to delegate an expanded evolution to itself
    _cell_factory(cell_factory), _cells(_height) {

    _map_center_x = std::floor((float)_width /2);
    _map_center_y = std::floor((float)_height/2);

    for (auto &row : _cells) {
      for (int i = 0; i < _width; i++) {
        row.push_back(cell_factory->create_cell());
      }
    }
    _remote_cell = cell_factory->create_cell();
  }

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  void update_cell(const DiscretePoint2D& cell_coord,
                   const Occupancy &new_value, double quality = 1.0) {
    // TODO: bounds check
    resize_if_need(cell_coord);
    int row = cell_coord.y + _map_center_y;
    int col = cell_coord.x + _map_center_x;
    _cells[row][col]->set_value(new_value, quality);
  }

  double cell_value(const DiscretePoint2D& cell_coord) const {
    if (has_cell(cell_coord)){
      int row = cell_coord.y+_map_center_y;
      int col = cell_coord.x+_map_center_x;
      return _cells[row][col]->value();
    }
    return _remote_cell->value();
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_x = std::floor(x/_m_per_cell);
    int cell_y = std::floor(y/_m_per_cell);

    return DiscretePoint2D(cell_x, cell_y);
  }

  double cell_scale() const { return _m_per_cell; }

  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    Rectangle bounds;
    bounds.bot = cell_coord.y * _m_per_cell;
    bounds.top = bounds.bot + _m_per_cell;
    bounds.left = cell_coord.x * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

  /*!
   * Gets the column number in container -
   * the \f$x\f$ coordinate of the map center
   */
  int get_map_center_x() const {return _map_center_x;}
  /*!
   * Gets the row number in container -
   * the \f$y\f$ coordinate of the map center
   */
  int get_map_center_y() const {return _map_center_y;}

private: // methods
  int expand_value(const int map_bound, const int abs_cell_coord) {
    if (abs_cell_coord < 0)
      return (-abs_cell_coord)+map_bound;
    if (map_bound <= abs_cell_coord)
      return abs_cell_coord + 1;
    return map_bound;
  }

  /* Finds the nearest upper bound value of degree of 2 to the input integer
   * NOTE: 2-3  are bounded by 4,
   *       4-7  are bounded by 8,
   *       8-15 are bounded by 16 etc.
   */
  long closest_bounded_power_two(long x) {
    long p2 = 1;
    while (p2 <= x){
      p2 <<= 1;
    }
    return p2;
  }

  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= (cell_coord.x + _map_center_x)          &&
                (cell_coord.x + _map_center_x) < _width &&
           0 <= (cell_coord.y + _map_center_y)          &&
                (cell_coord.y + _map_center_y) < _height;
  }

  void add_empty_rows(const int expand_value, GridMapContainer::iterator pos) {
    GridMapContainer empty_cells(expand_value, RowContainer(width()));
    for (auto& row : empty_cells)
      for (auto& cell : row)
        cell = _cell_factory->create_cell();

    _cells.insert(pos,
                  empty_cells.begin(),
                  empty_cells.end());
    _height += expand_value;
  }

  void add_empty_cols(const int expand_value,
                     std::function<RowContainer::iterator(RowContainer&)> pos) {
    GridMapContainer empty_cells(height(), RowContainer(expand_value));
    for (auto& row : empty_cells)
      for (auto& cell : row)
        cell = _cell_factory->create_cell();

    for(int i = 0; i < (int) empty_cells.size();i++)
      _cells[i].insert(pos(_cells[i]),
                       empty_cells[i].begin(),
                       empty_cells[i].end());
    _width += expand_value;
  }

  void resize(const int extra_value, const int direction_flags) {

    if(extra_value <= 0 || !(direction_flags & RESIZE_NEED))
      return;

    if (direction_flags & RESIZE_DOWN) {
      add_empty_rows(extra_value,_cells.begin());
      _map_center_y  += extra_value;
    }
    if (direction_flags & RESIZE_UP) {
      add_empty_rows(extra_value,_cells.end());
    }

    if (direction_flags & RESIZE_LEFT) {
      add_empty_cols(extra_value,
                     [](RowContainer& vector){return vector.begin();} );
      _map_center_x += extra_value;
    }
    if (direction_flags & RESIZE_RIGHT) {
      add_empty_cols(extra_value,
                     [](RowContainer& vector){return vector.end();} );
    }
  }

  void resize_if_need_in_line(const int directions_flag,
                              const int abs_cell_coord) {
    if(directions_flag != RESIZE_HORZ && directions_flag != RESIZE_VERT)
      return;
    int bound, flag;
    if (directions_flag == RESIZE_HORZ) {bound = width() ; flag = RESIZE_LEFT;}
    else                                {bound = height(); flag = RESIZE_DOWN;}
    //find if it needs to resize map
    int new_bound = expand_value(bound,abs_cell_coord);
    if (new_bound == bound)
      return;
    //expands map with the exponential rule
    // states are +100%, +300%, +700% etc ( -100% + [2^n*100%] )
    int expand_coef = closest_bounded_power_two(new_bound/bound) - 1;
    if (abs_cell_coord < 0) // resize the beginning of map vector
      resize(expand_coef*bound,flag);
    else
      resize(expand_coef*bound,flag<<1);
  }

  void resize_if_need(const DiscretePoint2D& cell_coord) {
    resize_if_need_in_line(RESIZE_HORZ,cell_coord.x+_map_center_x);
    resize_if_need_in_line(RESIZE_VERT,cell_coord.y+_map_center_y);
  }

private: // fields
  int _width, _height;
  int _map_center_x, _map_center_y;
  double _m_per_cell;
  Cell _remote_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
  GridMapContainer _cells;
};

#endif
