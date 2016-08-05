/**
 * \file
 * \brief There is 2 classes in this file.
 * GridCell - abstract base class, whose heirs return information about cell occupancy.
 * GridCellFactory - abstract base class, whose heirs can create a new cell.
 */

#ifndef __GRID_CELL_FACTORY_H
#define __GRID_CELL_FACTORY_H

#include <memory>

#include "cell_occupancy_estimator.h"

/**
 * \brief This is abstract class. Derived classes of this class calculate the probability that a cell is occupied.
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
 * \brief This class is abstract. Derived classes of this class create a new cell.
 */

class GridCellFactory {
public:
  virtual std::shared_ptr<GridCell> create_cell() = 0;
};

#endif
