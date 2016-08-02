#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

class GridMap {
public:
  using Cell = std::shared_ptr<GridCell>;
public:
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCellFactory> cell_factory):
    // TODO: replace hardcoded value with params
    _width(1), _height(1), _m_per_cell(0.1),
    // the map size 1x1 is chosen to delegate an expanded evolution to itself
    _cell_factory(cell_factory), _cells(_height) {

    _zero_x = _width /2;
    _zero_y = _height/2;

    for (auto &row : _cells) {
      for (int i = 0; i < _width; i++) {
        row.push_back(cell_factory->create_cell());
      }
    }
    _far_cell = cell_factory->create_cell();
  }

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  /*!
   * Updates the cell probability to be occupied with a new occupancy value
   * If the cell with the input coordinates do not exist in the current map then
   * the map is resized.
   */
  void update_cell(const DiscretePoint2D& cell_coord,
                   const Occupancy &new_value, double quality = 1.0) {
    // TODO: bounds check
    int cell_x = cell_coord.x;
    int cell_y = cell_coord.y;
    int width_add, height_add;
    //find if it needs to resize map
    width_add  = need_add(cell_x, _zero_x, width());
    height_add = need_add(cell_y, _zero_y, height());

    //expands map with the exponential rule
    // states are +100%, +300%, +700% etc ( -100% + [2^n*100%] )
    if (width_add != 0) {
      int expand_koef = clp2(width_add/width()+1)-1;
      if (cell_x+_zero_x < 0)
        resize(expand_koef*width(),4); //flag 0b0100 - left
      else
        resize(expand_koef*width(),8); //flag 0b1000 - right
    }
    if (height_add != 0) {
      int expand_koef = clp2(height_add/height()+1)-1;
      if (cell_y+_zero_y < 0)
        resize(expand_koef*height(),1); //flag 0b0001 - down
      else
        resize(expand_koef*height(),2); //flag 0b0010 - up
    }
    _cells[cell_y+_zero_y][cell_x+_zero_x]->set_value(new_value, quality);
  }

  double cell_value(const DiscretePoint2D& cell_coord) const {
    if (has_cell(cell_coord))
      return _cells[cell_coord.y+_zero_y][cell_coord.x+_zero_x]->value();
    else
      return _far_cell->value();
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

  int get_zero_x() const {return _zero_x;}
  int get_zero_y() const {return _zero_y;}
private: // methods
  int need_add(const int cell_coord, const int zero_pos, const int map_bound) {
    if (cell_coord+zero_pos < 0)
      return -(cell_coord+zero_pos);
    if (cell_coord+zero_pos >= map_bound)
      return (cell_coord+zero_pos) - map_bound + 1;
    return 0;
  }
  /* Finds the nearest upper bound value of degree of 2 to the input integer
   * clp2(0) = 1
   * clp2(1) = 2
   * clp2(2) = 4
   * clp2(3) = 4
   * clp2(4) = 8
   * clp2(5) = 8
   */
  long clp2(long x) {
    long p2=1;
    while (p2<=x){
      p2 <<= 1;
    }
    return p2;
  }

  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= (cell_coord.x + _zero_x) && (cell_coord.x + _zero_x) < _width &&
           0 <= (cell_coord.y + _zero_y) && (cell_coord.y + _zero_y) < _height;
  }

  using RowContainer = std::vector<Cell>;
  using GridMapContainer = std::vector<RowContainer>;

  enum class Direction {UP,DOWN,LEFT,RIGHT};

  void add_empty_cells(const Direction& dir,
                       const int empty_height = 0,
                       const int empty_width  = 0) {

    GridMapContainer empty_cells(empty_height, RowContainer(empty_width));
    for (auto& row : empty_cells)
      for (auto& cell : row)
        cell = _cell_factory->create_cell();

    switch (dir) {
      case Direction::DOWN:
        _cells.insert(_cells.begin(),empty_cells.begin(), empty_cells.end());
        _zero_y  += empty_height;
        _height += empty_height;
        break;

      case Direction::UP:
        _cells.insert(_cells.end(),empty_cells.begin(), empty_cells.end());
        _height += empty_height;
        break;

      case Direction::LEFT:
        for (int i = 0; i < height(); i++)
          _cells[i].insert(_cells[i].begin(),
                           empty_cells[i].begin(),empty_cells[i].end());
        _zero_x += empty_width;
        _width += empty_width;
        break;

      case Direction::RIGHT:
        for (int i = 0; i < height(); i++)
          _cells[i].insert(_cells[i].end(),
                           empty_cells[i].begin(),empty_cells[i].end());
        _width += empty_width;
        break;

      default:
        break;
    }
  }

  void resize(const int add_value, const int flags_direction) {
    if(add_value <= 0 || !(flags_direction & 15))
      return;

    const int RESIZE_DOWN     = 1; // 0b0001
    const int RESIZE_UP       = 2; // 0b0010
    const int RESIZE_LEFT     = 4; // 0b0100
    const int RESIZE_RIGHT    = 8; // 0b1000

    if (flags_direction & RESIZE_DOWN) {
      add_empty_cells(Direction::DOWN, add_value, width());
    }
    if (flags_direction & RESIZE_UP) {
      add_empty_cells(Direction::UP,   add_value, width());
    }

    if (flags_direction & RESIZE_LEFT) {
      add_empty_cells(Direction::LEFT,  height(),add_value);
    }
    if (flags_direction & RESIZE_RIGHT) {
      add_empty_cells(Direction::RIGHT, height(),add_value);
    }
  }

private: // fields
  int _width, _height;
  int _zero_x, _zero_y;
  double _m_per_cell;
  Cell _far_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
  std::vector<std::vector<Cell>> _cells;
};

#endif
