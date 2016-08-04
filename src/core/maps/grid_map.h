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

/**
 * \brief An occupancy grid implementation.
 */
class GridMap {
public:
  using Cell = std::shared_ptr<GridCell>;
public:
  // TODO: cp, mv ctors, dtor
  /**
   * Creates a GridCell based map.
   * \param cell_factory The factory that a creates requied type of Cell.
   */
  GridMap(std::shared_ptr<GridCellFactory> cell_factory):
    // TODO: replace hardcoded value with params
    _width(500), _height(500), _m_per_cell(0.1),
    _cell_factory(cell_factory), _cells(_height) {
    for (auto &row : _cells) {
      for (int i = 0; i < _width; i++) {
        row.push_back(cell_factory->create_cell());
      }
    }
  }

  /// Returns the width of map.
  int width() const { return _width; }

  /// Returns thr height of map.
  int height() const { return _height; }

  /// Returns the acale.
  double scale() const { return _m_per_cell; }

  /// Returns map's cells.
  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  /**
   * Updates a cell with a new occupancy data.
   * \param cell_coord Coordinates of a cell.
   * \param new_value The probability for cell of being occupied.
   * \param quality The Measure of beleif to the data.
   */
  void update_cell(const DiscretePoint2D& cell_coord,
                   const Occupancy &new_value, double quality = 1.0) {
    // TODO: bounds check
    _cells[cell_coord.y][cell_coord.x]->set_value(new_value, quality);
  }

  /**
   * Returns the probability of the cell to be occupied.
   * \param cell_coord A point with coordinates of requied cell in Grid Map.
   */
  double cell_value(const DiscretePoint2D& cell_coord) const {
    return _cells[cell_coord.y][cell_coord.x]->value();
  }

  /**
   * Returns a constant reference on a cell by the point.
   * \param cell_coord The point with coordinates of requied cell in Grid Map.
   */
  const Cell &cell(const DiscretePoint2D& cell_coord) const {
    return _cells[cell_coord.y][cell_coord.x];
  }

  /**
   * Returns a cell by the coordinates.
   * \param x,y Coordinates of a cell in Grid Map in cartesian coordinates.
   */
  const Cell &cell(int x, int y) const {
      return _cells[x][y];
  }

  /**
   * Returns a reference on cell by the point.
   * \param cell_coord The point with coordinates of requied cell in Grid Map.
   */
  Cell &cell(const DiscretePoint2D& cell_coord) {
    return _cells[cell_coord.y][cell_coord.x];
  }

  /**
   * Tests whether the point is contained in a map.
   * \param cell_coord The point with coordinates of requied cell in Grid Map.
   */
  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.y < _width &&
           0 <= cell_coord.y && cell_coord.y < _height;
  }

  /**
   * Projects coordinates in meters onto a cell of the grid map.
   * \return The cell point corresponding to a given coordinates.
   */
  DiscretePoint2D world_to_cell(double x, double y) const {
    int cell_x = std::floor(_width /2 + x/_m_per_cell);
    int cell_y = std::floor(_height/2 + y/_m_per_cell);

    return DiscretePoint2D(cell_x, cell_y);
  }

  /// Returnes the scale.
  double cell_scale() const { return _m_per_cell; }

  /**
   * Returns the bounds of the cell in the World.
   * \param cell_coord The point with coordinates of requied cell in Grid Map.
   */
  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    Rectangle bounds;
    bounds.bot = (cell_coord.y + _height/2) * _m_per_cell;
    bounds.top = bounds.bot + _m_per_cell;
    bounds.left = (cell_coord.x + _width/2) * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
  std::vector<std::vector<Cell>> _cells;
};

#endif
