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
    return equal(prob_occ, that.prob_occ) &&
           equal(estimation_quality, that.estimation_quality);
  }
  void setNan(){
    prob_occ = std::numeric_limits<double>::quiet_NaN();
    estimation_quality = std::numeric_limits<double>::quiet_NaN();
  }
  bool isNan(){
    return std::isnan(prob_occ) || std::isnan(estimation_quality);
  }
};

struct Beam {
  double x_st, y_st;
  double x_end, y_end;

  Beam(double x_st, double y_st, double x_end, double y_end){
    this->x_st = x_st; this->y_st = y_st; this->x_end = x_end; this->y_end = y_end;
  }
  //methods
  bool tangents_rect(const Rectangle& bnds) {
    return equal(x_st, x_end, bnds.left) || equal(y_st, y_end, bnds.bot);
  }

  bool intersects_rect(const Rectangle& bnds) {
    bool result = false;
    if(bnds.contains(x_st,y_st) || bnds.contains(y_st,y_end)) {
      return true;
    }
    if (equal(x_st, x_end)) {
      return(is_between(bnds.bot, y_st, y_end) &&
             is_between(x_st, bnds.left, bnds.right));
    }
    if(both_less(x_st, x_end, bnds.left) ||
       both_higher(x_st, x_end, bnds.right) ||
       both_less(y_st, y_end, bnds.bot) ||
       both_higher(y_st, y_end, bnds.top)) {
         return false;
    }
    double k = (y_end - y_st) / (x_end - x_st);
    double b = y_st - ((x_st * (y_end - y_st)) / (x_end - x_st));  // y = kx + b
    double inters_x, inters_y;
    inters_x = bnds.left;
    result = is_between((k * inters_x) + b, bnds.bot, bnds.top);
    inters_x = bnds.right;
    result |= is_between((k * inters_x) + b, bnds.bot, bnds.top);
    inters_y = bnds.bot;
    result |= is_between((inters_y-b)/k, bnds.left, bnds.right);
    inters_y = bnds.top;
    result |= is_between((inters_y-b)/k, bnds.left, bnds.right);
    return result;
  }
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
