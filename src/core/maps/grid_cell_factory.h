#ifndef __GRID_CELL_FACTORY_H
#define __GRID_CELL_FACTORY_H

#include <memory>

#include "cell_occupancy_estimator.h"

class GridCell {
public:
  virtual double value() const = 0;
  virtual void set_value(const Occupancy &occ, double quality = 1.0) = 0;

  // estimated obstacle center
  virtual double obst_x() const { return 0; }
  virtual double obst_y() const { return 0; }
};

class GridCellFactory {
public:
  virtual std::shared_ptr<GridCell> create_cell() = 0;
};

#endif
