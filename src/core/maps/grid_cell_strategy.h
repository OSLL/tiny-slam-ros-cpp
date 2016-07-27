
#ifndef __GRID_CELL_STRATEGY_H
#define __GRID_CELL_STRATEGY_H

#include <memory>

#include "grid_cell_factory.h"
#include "../grid_scan_matcher.h"
#include "cell_occupancy_estimator.h"

/**
 * \brief This class is responsible for strategy of cell's creation.
 */

class GridCellStrategy {
public:
  GridCellStrategy(std::shared_ptr<GridCellFactory> factory,
                   std::shared_ptr<ScanCostEstimator> cost_est,
                   std::shared_ptr<CellOccupancyEstimator> occ_est)
    : _cell_factory(factory), _cost_estimator(cost_est),
      _occupancy_estimator(occ_est) {}

/**
 * Gives information about strategy of cell creating.
 * \return Pointer on GridCellFactory.
 */

  std::shared_ptr<GridCellFactory> cell_factory() { return _cell_factory; }

/**
 * Gives information about scan's cost.
 * \return Pointer on ScanCostEstimator.
 */

  std::shared_ptr<ScanCostEstimator> cost_est() { return _cost_estimator; }

/**
 * Gives information about strategy of new cell's occupancy.
 * \return Pointer on CellOccupancyEstimator.
 */

  std::shared_ptr<CellOccupancyEstimator> occupancy_est() {
    return _occupancy_estimator;
  }

private:
  // cell creation/update
  std::shared_ptr<GridCellFactory> _cell_factory;
  // cell comparison (scan evaluation)
  std::shared_ptr<ScanCostEstimator> _cost_estimator;
  // new cell value estimation
  std::shared_ptr<CellOccupancyEstimator> _occupancy_estimator;
};

#endif
