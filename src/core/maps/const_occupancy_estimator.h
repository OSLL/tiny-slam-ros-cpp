/**
 * \file
 * \brief There is the class in this file
 */

#ifndef __CONST_OCCUPANCY_ESTIMATOR_H
#define __CONST_OCCUPANCY_ESTIMATOR_H

#include "cell_occupancy_estimator.h"

/**
 * \brief This class gives the information about cell's occupancy.
 */

class ConstOccupancyEstimator : public CellOccupancyEstimator {
public:
  ConstOccupancyEstimator(double occ, double empty) :
    CellOccupancyEstimator(occ, empty) {}

  /**
   * This method returns information about cell's occupancy.
   */

  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    return Occupancy{is_occ ? base_occ_prob() : base_empty_prob(), 1.0};
  }

};

#endif
