#ifndef __GRID_CELL_STRATEGY_H
#define __GRID_CELL_STRATEGY_H

#include <memory>

#include "grid_cell_factory.h"
#include "../grid_scan_matcher.h"
#include "cell_occupancy_estimator.h"

/**
 * \brief A container for strategies specific to a grid cell model.
 */
class GridCellStrategy {
public:
/**
 * \brief Initializes a grid cell model.
 * \param factory The Cell factory that creates cells of a specific type.
 * \param cost_est A scan cost estimator.
 * \param occ_ess An occupancy estimator.
 */
  GridCellStrategy(std::shared_ptr<GridCellFactory> factory,
                   std::shared_ptr<ScanCostEstimator> cost_est,
                   std::shared_ptr<CellOccupancyEstimator> occ_est)
    : _cell_factory(factory), _cost_estimator(cost_est),
      _occupancy_estimator(occ_est) {}

/**
 * \brief Returns a cell factory that creates cells that implement a specific model.
 * \return Pointer to GridCellFactory.
 */
  std::shared_ptr<GridCellFactory> cell_factory() { return _cell_factory; }

/**
 * \brief Returns a scan cost estimator.
 * \return Pointer to ScanCostEstimator.
 */
  std::shared_ptr<ScanCostEstimator> cost_est() { return _cost_estimator; }

/**
 * \brief Returns an occupancy estimator.
 * \return Pointer to CellOccupancyEstimator.
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
