#ifndef __CONST_OCCUPANCY_ESTIMATOR_H
#define __CONST_OCCUPANCY_ESTIMATOR_H

#include "cell_occupancy_estimator.h"

/**
 * \brief ConstEstimator is responsible for occupancy estimation strategy
 * that returns probability of being occupied for a cell depending only
 * on the cell status predicted by a sensor.
 */

class ConstOccupancyEstimator : public CellOccupancyEstimator {
public:
/**
 * Initializes the estimator with base probabilities.
 * \param occ probability of being occupied.
 * \param empty probability of being empty.
 */
  ConstOccupancyEstimator(double occ, double empty) :
    CellOccupancyEstimator(occ, empty) {}
/**
 * Returns base probability of being occupied for the cell defined by bounds.
 * \param beam Laser beam (ignored).
 * \param cell_bnds geometric representation of the cell (ignored).
 * \param is_occ flag that indicates whether the cell is occupied according to sensor data.
 */
  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    return Occupancy{is_occ ? base_occ_prob() : base_empty_prob(), 1.0};
  }

};

#endif
