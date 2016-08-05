/**
 * \file
 * \brief The following classes are defined in this file
 * GridCell - base class for GridMap's cell;
 * GridCellFactory - abstract base class which subclasses create a new cell of a specific type.
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
/**
 *\brief Returns the probability of being occupied for a cell.
 */
  virtual double value() const = 0;
/**
 * \brief Calculates the probability of being occupied for a cell.
 */
  virtual void set_value(const Occupancy &occ, double quality = 1.0) = 0;

  // TODO: move to gmapping
  virtual double obst_x() const { return 0; }
  virtual double obst_y() const { return 0; }
};

/**
 * \brief The base class for factories that encapsulate creation of a specific cell
 * (The Factory method pattern is applied).
 */
class GridCellFactory {
public:
/**
 * \brief Creates a new cell of a specific type.
 * \return Pointer to GridCell.
 */
  virtual std::shared_ptr<GridCell> create_cell() = 0;
};

#endif
