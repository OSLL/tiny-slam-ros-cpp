#ifndef __GRID_CELL_STRATEGY_H
#define __GRID_CELL_STRATEGY_H

#include <memory>

#include "grid_cell_factory.h"
#include "../grid_scan_matcher.h"
#include "cell_occupancy_estimator.h"

class GridCellStrategy {
public:
  GridCellStrategy(std::shared_ptr<GridCellFactory> factory,
                   std::shared_ptr<ScanCostEstimator> cost_est,
                   std::shared_ptr<CellOccupancyEstimator> occ_est)
    : _cell_factory(factory), _cost_estimator(cost_est),
      _occupancy_estimator(occ_est) {}

  std::shared_ptr<GridCellFactory> cell_factory() { return _cell_factory; }
  std::shared_ptr<ScanCostEstimator> cost_est() { return _cost_estimator; }
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
