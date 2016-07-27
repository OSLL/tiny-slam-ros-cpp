
#ifndef __CONST_OCCUPANCY_ESTIMATOR_H
#define __CONST_OCCUPANCY_ESTIMATOR_H

#include "cell_occupancy_estimator.h"

/**
 * \brief This class is responsible for strategy of base occupancy estimation.
 */

class ConstOccupancyEstimator : public CellOccupancyEstimator {
public:
  ConstOccupancyEstimator(double occ, double empty) :
    CellOccupancyEstimator(occ, empty) {}

/**
 * This virtual method returns base probability that cell is occupied;
 * ignores fist two parameters cause in this class implemented strategy of base occupancy estimation.
 */

  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    return Occupancy{is_occ ? base_occ_prob() : base_empty_prob(), 1.0};
  }

};

#endif
