#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

class GridMap {
private: //flag constants
  const int RESIZE_VERT    = 0;
  const int RESIZE_HORZ    = 1;
  const int RESIZE_DIM_BIT = 1;

  const int RESIZE_FWD     = 0;
  const int RESIZE_BWD     = 1;
  const int RESIZE_DIR_BIT = 0;

  #define RESIZE_DIR(DIM,DIR)                            \
    ((DIM) << RESIZE_DIM_BIT) | ((DIR) << RESIZE_DIR_BIT)

  const int RESIZE_UP    = RESIZE_DIR(RESIZE_VERT, RESIZE_FWD);
  const int RESIZE_DOWN  = RESIZE_DIR(RESIZE_VERT, RESIZE_BWD);
  const int RESIZE_RIGHT = RESIZE_DIR(RESIZE_HORZ, RESIZE_FWD);
  const int RESIZE_LEFT  = RESIZE_DIR(RESIZE_HORZ, RESIZE_BWD);
public: // typedefs
  using Cell = std::shared_ptr<GridCell>;
private: // typedefs
  using Row = std::vector<Cell>;
  using Map = std::vector<Row>;

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
    _unvisited_cell = cell_factory->create_cell();
  }

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  #define INDEX_J(cell_coord)     \
      (cell_coord.x) + _map_center_x
  #define INDEX_I(cell_coord)     \
      (cell_coord.y) + _map_center_y

  void update_cell(const DiscretePoint2D& cell_coord,
                   const Occupancy &new_value, double quality = 1.0) {
    // TODO: bounds check
    update_size(cell_coord);

    int row = INDEX_I(cell_coord);
    int col = INDEX_J(cell_coord);
    _cells[row][col]->set_value(new_value, quality);
  }

  double cell_value(const DiscretePoint2D& cell_coord) const {
    if (!has_cell(cell_coord))
      return _unvisited_cell->value();

    return _cells[INDEX_I(cell_coord)][INDEX_J(cell_coord)]->value();
  }

  DiscretePoint2D world_to_cell(double x, double y) const {
    #define METERS_TO_CELLS(var) \
      std::floor((var)/_m_per_cell)

    return DiscretePoint2D(METERS_TO_CELLS(x), METERS_TO_CELLS(y));
    #undef METERS_TO_CELLS
  }

  double cell_scale() const { return _m_per_cell; }

  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    Rectangle bounds;
    bounds.bot   = cell_coord.y * _m_per_cell;
    bounds.top   = bounds.bot + _m_per_cell;
    bounds.left  = cell_coord.x * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

  /*!
   * Returns the column index in container -
   * the \f$x\f$ coordinate of the map center
   */
  int map_center_x() const {return _map_center_x;}
  /*!
   * Returns the row index in container -
   * the \f$y\f$ coordinate of the map center
   */
  int map_center_y() const {return _map_center_y;}

private: // methods
  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= INDEX_J(cell_coord) && INDEX_J(cell_coord) < _width &&
           0 <= INDEX_I(cell_coord) && INDEX_I(cell_coord) < _height;
  }

  void update_size(const DiscretePoint2D& cell_coord) {
    resize_bound(RESIZE_HORZ, cell_coord.x+_map_center_x);
    resize_bound(RESIZE_VERT, cell_coord.y+_map_center_y);
  }

  void resize_bound(const int dim, const int container_coord) {
    if (dim != RESIZE_HORZ && dim != RESIZE_VERT)
      return;

    int bound = (dim == RESIZE_HORZ ? width() : height());
    //find if it needs to resize map
    int new_bound = calc_sufficient_bound(container_coord, bound);
    if (new_bound == bound)
      return;
    //expands map with the exponential rule
    // states are +100%, +300%, +700% etc ( -100% + [2^n*100%] )
    int expand_coef = closest_bounded_power_two(new_bound/bound) - 1;
    resize_in_direction(expand_coef*bound,RESIZE_DIR(dim,container_coord<0) );
  }

  int calc_sufficient_bound(const int container_coord, const int map_bound) {
    if (container_coord < 0)
      return (-container_coord)+map_bound;
    if (map_bound <= container_coord)
      return container_coord + 1;
    return map_bound;
  }

  /* Finds the nearest upper bound value of degree of 2 to the input integer
   * NOTE: 2-3  are bounded by 4,
   *       4-7  are bounded by 8,
   *       8-15 are bounded by 16 etc.
   */
  long closest_bounded_power_two(long x) {
    long p2 = 1;
    while (p2 <= x) {
      p2 <<= 1;
    }
    return p2;
  }

  void resize_in_direction(const int delta, const int direction_flag) {
    if (delta <= 0)
      return;

    if (direction_flag == RESIZE_DOWN) {
      add_empty_rows(delta, _cells.begin());
      _map_center_y  += delta;
    } else if (direction_flag == RESIZE_UP) {
      add_empty_rows(delta, _cells.end());
    } else if (direction_flag == RESIZE_LEFT) {
      add_empty_cols(delta, [](Row& vector) { return vector.begin(); });
      _map_center_x += delta;
    } else if (direction_flag == RESIZE_RIGHT) {
      add_empty_cols(delta, [](Row& vector) { return vector.end(); });
    }
  }

  void add_empty_rows(const int expand_value, Map::iterator it) {
    Map zero_rows(expand_value, Row(width()));
    for (auto& row : zero_rows) {
      for (auto& cell : row) {
        cell = _cell_factory->create_cell();
      }
    }
    _cells.insert(it, zero_rows.begin(), zero_rows.end());
    _height += expand_value;
  }

  void add_empty_cols(const int expand_value,
                      std::function<Row::iterator(Row&)> it) {
    Map zero_cols(height(), Row(expand_value));
    for (auto& row : zero_cols) {
      for (auto& cell : row) {
        cell = _cell_factory->create_cell();
      }
    }
    for (size_t i = 0; i < zero_cols.size(); i++) {
      _cells[i].insert(it(_cells[i]), zero_cols[i].begin(), zero_cols[i].end());
    }
    _width += expand_value;
  }
  #undef INDEX_I
  #undef INDEX_J
  #undef RESIZE_DIR
private: // fields
  int _width, _height;
  int _map_center_x, _map_center_y;
  double _m_per_cell;
  Cell _unvisited_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
  Map _cells;
};

#endif
