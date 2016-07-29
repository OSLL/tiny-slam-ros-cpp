/**
 * \file
 * The following classes are defined in this file
 * GridCell - base class for GridMap's cell;
 * GridCellFactory - abstract base class which subclasses create a new cell specific type.
 */

#ifndef __GRID_CELL_FACTORY_H
#define __GRID_CELL_FACTORY_H

#include <memory>

#include "cell_occupancy_estimator.h"

/**
 * \brief The base class for GridMap's cell that defines a model of occupancy tracking.
 */
class GridCell {
public:
  virtual double value() const = 0;
  virtual void set_value(const Occupancy &occ, double quality = 1.0) = 0;

  // estimated obstacle center
  virtual double obst_x() const { return 0; }
  virtual double obst_y() const { return 0; }
};

/**
 * \brief The base class for factories that encapsulate creation of a specific cell (Factory method pattern is applied).
 */
class GridCellFactory {
public:
  virtual std::shared_ptr<GridCell> create_cell() = 0;
};

#endif
