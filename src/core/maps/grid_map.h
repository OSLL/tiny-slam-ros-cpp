/**
 * \file
 * \brief Defines the Grid map.
 */
#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

struct GridMapParams {
  double width, height, meters_per_cell; // width & height in meters
};

/**
 * \brief An occupancy grid implementation.
 */
class GridMap {
public: // typedefs
  using Cell = std::shared_ptr<GridCell>;
private: // typedefs
  using Row = std::vector<Cell>;
  using Map = std::vector<Row>;

public:
  // TODO: cp, mv ctors, dtor
  /**
   * Creates a GridCell based map.
   * \param cell_factory The factory that creates a requied type of Cell.
   */
  GridMap(std::shared_ptr<GridCellFactory> cell_factory,
          const GridMapParams &init_params) :
    _width(std::floor(init_params.width/init_params.meters_per_cell)),
    _height(std::floor(init_params.height/init_params.meters_per_cell)),
    _m_per_cell(init_params.meters_per_cell),
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

  /// Returns the width of the map.
  int width() const { return _width; }

  /// Returns thr height of the map.
  int height() const { return _height; }

  /// Returns the scale.
  double scale() const { return _m_per_cell; }

  /// Returns map's cells.
  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  #define COL_IND(cell_coord)         \
      ((cell_coord).x + _map_center_x)
  #define ROW_IND(cell_coord)         \
      ((cell_coord).y + _map_center_y)

  /**
   * Updates a cell with a new occupancy data.
   * \param cell_coord Coordinates of a cell.
   * \param new_value The probability for cell of being occupied.
   * \param quality The Measure of beleif to the data.
   */
  void update_cell(const DiscretePoint2D& cell_coord,
                   const Occupancy &new_value, double quality = 1.0) {
    // TODO: bounds check
    update_size(cell_coord);

    int row = ROW_IND(cell_coord);
    int col = COL_IND(cell_coord);
    _cells[row][col]->set_value(new_value, quality);
  }

  /**
   * Returns the probability of the cell to be occupied.
   * \param cell_coord A point with coordinates of requied cell in Grid Map.
   */
  double cell_value(const DiscretePoint2D& cell_coord) const {
    if (!has_cell(cell_coord))
      return _unvisited_cell->value();

    return _cells[ROW_IND(cell_coord)][COL_IND(cell_coord)]->value();
  }

  /**
   * Projects coordinates in meters onto a cell of the grid map.
   * \return The cell point corresponding to given coordinates.
   */
  DiscretePoint2D world_to_cell(double x, double y) const {
    #define METERS_TO_CELLS(var)   \
      std::floor((var)/_m_per_cell)

    return DiscretePoint2D(METERS_TO_CELLS(x), METERS_TO_CELLS(y));
    #undef METERS_TO_CELLS
  }

  /// Returnes the scale.
  double cell_scale() const { return _m_per_cell; }

  /**
   * Returns the bounds of the cell in the World.
   * \param cell_coord The point with coordinates of requied cell in the grid
   *                                                                       map.
   */
  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    Rectangle bounds;
    bounds.bot   = cell_coord.y * _m_per_cell;
    bounds.top   = bounds.bot + _m_per_cell;
    bounds.left  = cell_coord.x * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

  /*!
   * Returns the \f$x\f$ coordinate of the map center
   */
  int map_center_x() const { return _map_center_x; }
  /*!
   * Returns the \f$y\f$ coordinate of the map center
   */
  int map_center_y() const { return _map_center_y; }

private: // methods
  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= COL_IND(cell_coord) && COL_IND(cell_coord) < _width &&
           0 <= ROW_IND(cell_coord) && ROW_IND(cell_coord) < _height;
  }

  void update_size(const DiscretePoint2D& cell_coord) {
    resize_bound(RESIZE_HORZ, COL_IND(cell_coord));
    resize_bound(RESIZE_VERT, ROW_IND(cell_coord));
  }

  #define RESIZE_DIR(DIM,DIR)                              \
    (((DIM) << RESIZE_DIM_BIT) | ((DIR) << RESIZE_DIR_BIT))

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
    int expand_rate = closest_bounded_power_two(new_bound/bound) - 1;
    resize_in_direction(expand_rate*bound, RESIZE_DIR(dim, container_coord<0));
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
      add_rows(delta, _cells.begin());
      _map_center_y  += delta;
    } else if (direction_flag == RESIZE_UP) {
      add_rows(delta, _cells.end());
    } else if (direction_flag == RESIZE_LEFT) {
      add_cols(delta, [](Row& vector) { return vector.begin(); });
      _map_center_x += delta;
    } else if (direction_flag == RESIZE_RIGHT) {
      add_cols(delta, [](Row& vector) { return vector.end(); });
    }
  }

  void add_rows(const int rows_count, Map::iterator it) {
    Map new_rows(rows_count, Row(width()));
    for (auto& row : new_rows) {
      for (auto& cell : row) {
        cell = _cell_factory->create_cell();
      }
    }
    _cells.insert(it, new_rows.begin(), new_rows.end());
    _height += rows_count;
  }

  void add_cols(const int cols_count, std::function<Row::iterator(Row&)> it) {
    Map new_cols(height(), Row(cols_count));
    for (auto& row : new_cols) {
      for (auto& cell : row) {
        cell = _cell_factory->create_cell();
      }
    }
    for (size_t i = 0; i < new_cols.size(); i++) {
      _cells[i].insert(it(_cells[i]), new_cols[i].begin(), new_cols[i].end());
    }
    _width += cols_count;
  }
  #undef INDEX_I
  #undef INDEX_J

private: //flag constants
  const int RESIZE_VERT    = 0;
  const int RESIZE_HORZ    = 1;
  const int RESIZE_DIM_BIT = 1;

  const int RESIZE_FWD     = 0;
  const int RESIZE_BWD     = 1;
  const int RESIZE_DIR_BIT = 0;

  const int RESIZE_UP    = RESIZE_DIR(RESIZE_VERT, RESIZE_FWD);
  const int RESIZE_DOWN  = RESIZE_DIR(RESIZE_VERT, RESIZE_BWD);
  const int RESIZE_RIGHT = RESIZE_DIR(RESIZE_HORZ, RESIZE_FWD);
  const int RESIZE_LEFT  = RESIZE_DIR(RESIZE_HORZ, RESIZE_BWD);
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
