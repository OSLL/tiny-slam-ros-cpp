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
    estimation_quality = 0.01;
  }
  bool isNan(){
    return std::isnan(prob_occ);
  }
};

struct Beam {
  double x_st, y_st;
  double x_end, y_end;

  //methods
  bool tangents_rect(const Rectangle& bnds) const {
    return equal(x_st, x_end, bnds.left) || equal(y_st, y_end, bnds.bot) ||
           equal(x_st, x_end, bnds.right) || equal(y_st, y_end, bnds.top);
  }

  bool intersects_rect(const Rectangle& bnds) const {
    bool result = false;
    if(bnds.contains(x_st, y_st) || bnds.contains(x_end, y_end)) {
      return true;
    }
    if (equal(x_st, x_end)) {
      return(is_between(bnds.bot, y_st, y_end) &&
             is_strictly_between(x_st, bnds.left, bnds.right));
    }

    if(both_less(x_st, x_end, bnds.left) ||
       both_higher(x_st, x_end, bnds.right) ||
       both_less(y_st, y_end, bnds.bot) ||
       both_higher(y_st, y_end, bnds.top)) {
         return false;
    }
    double k = (y_end - y_st) / (x_end - x_st);
    double b = y_st - ((x_st * (y_end - y_st)) / (x_end - x_st)); // y = kx + b
    double inters_x, inters_y;
    bool cross_left_bot, cross_left_top, cross_right_bot, cross_right_top;

    inters_x = bnds.left;
    inters_y = (k * inters_x) + b;
    cross_left_top = equal(inters_y, bnds.top);
    cross_left_bot = equal(inters_y, bnds.bot);
    result = is_strictly_between(inters_y, bnds.bot, bnds.top);

    inters_x = bnds.right;
    inters_y = (k * inters_x) + b;
    result |= is_strictly_between(inters_y, bnds.bot, bnds.top);
    cross_right_top = equal(inters_y, bnds.top);
    cross_right_bot = equal(inters_y, bnds.bot);

    inters_y = bnds.bot;
    result |= is_strictly_between((inters_y-b)/k, bnds.left, bnds.right);

    inters_y = bnds.top;
    result |= is_strictly_between((inters_y-b)/k, bnds.left, bnds.right);
    if((cross_left_bot && cross_right_top) ||
       (cross_left_top && cross_right_bot)) {
      return true;
    }
    return result;
  }

  bool encounters_rect(const Rectangle& bnds) const {
    if(!bnds.contains_on_border(x_end, y_end) || tangents_rect(bnds)) {
      return false;
    }
    if (equal(x_end, bnds.left) || equal(x_end, bnds.right)) {
      return is_strictly_between(x_end, x_st, bnds.center_x());
    }
    if (equal(y_end, bnds.bot) || equal(y_end, bnds.top)) {
      return is_strictly_between(y_end, y_st, bnds.center_y());
    }
    return false;
  }

  bool reaches_border_passing_through_rect(const Rectangle& bnds) const {
    return bnds.contains_on_border(x_end, y_end) && intersects_rect(bnds);
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
