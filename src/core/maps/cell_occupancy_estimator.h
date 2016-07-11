#ifndef _CELL_OCCUPANCY_ESTIMATOR_H
#define _CELL_OCCUPANCY_ESTIMATOR_H

#include "../geometry_utils.h"

struct Occupancy {
  double prob_occ;
  double estimation_quality;
  // estimated obstacle center
  double x, y;

  Occupancy(double prob, double quality) :
    prob_occ(prob), estimation_quality(quality) {}

  Occupancy(double prob, double quality, double curr_x, double curr_y) :
    prob_occ(prob), estimation_quality(quality), x(curr_x), y(curr_y) {}

  bool operator==(const Occupancy &that) {
    return EQ_DOUBLE(prob_occ, that.prob_occ) &&
           EQ_DOUBLE(estimation_quality, that.estimation_quality);
  }
};

struct Beam {
  double x_st, y_st;
  double x_end, y_end;
};

class CellOccupancyEstimator {
public:
  CellOccupancyEstimator(double base_occ_prob, double base_empty_prob):
    _base_occ_prob(base_occ_prob), _base_empty_prob(base_empty_prob) {}
  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) = 0;
protected:
  double base_occ_prob() { return _base_occ_prob; }
  double base_empty_prob() { return _base_empty_prob; }
private:
  double _base_occ_prob, _base_empty_prob;
};

#endif
