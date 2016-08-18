/**
 * \file
 * \brief Defines interface Cell Occupancy Estimator.
 * There are structures Occupancy and Beam that are used in class
 * CellOcupancyEstimator.
 */
#ifndef _CELL_OCCUPANCY_ESTIMATOR_H
#define _CELL_OCCUPANCY_ESTIMATOR_H

#include "../geometry_utils.h"

/**
 * Contains the information about a probability of occupancy and a quality of
 * estimation for 2D point in cartesian coordinates(x,y).
 */
struct Occupancy {
  double prob_occ;
  double estimation_quality;
  double x, y; ///< The estimated obstacle center

  /**
   * Initializes a probability of occupation and a quality of estimation.
   * \param A probability of occupation of a current cell.
   * \param quality A quality of estimation for a current cell.
   */
  Occupancy(double prob, double quality) :
    prob_occ(prob), estimation_quality(quality) {}

  /**
   * Initializes a probability of occupation and a quality of estimation for a
   * point in cartesian coordinates (x,y).
   * \param prob A probability of occupation of current cell.
   * \param quality A quality of estimation for current cell.
   * \param x,y Coordinates of an occupied cell
   */
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

  bool isNan() const {
    return std::isnan(prob_occ) || std::isnan(estimation_quality);
  }
};
/**
 * The representation of a laser beam bounded by an obstacle.
 */
struct Beam {
  double x_st, y_st; ///< Coordinates of a start of the beam.
  double x_end, y_end; ///< Coordinates where the beam reaches an obstacle.

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
  Beam generate_revert() const {
    Beam result(*this);
    std::swap(result.x_st, result.x_end);
    std::swap(result.y_st, result.y_end);
    return result;
  }
};

/**
 * Estimates the occupancy of cell according to a strategy defined by
 * subclasses.
 */
class CellOccupancyEstimator {
public:

  /**
   * Initializes an estimator with probability of cell to be occupied and empty.
   * \param base_occ_prob The initial probability of a cell to be occupied.
   * \param base_empty_prob The initial probability of a cell to be empty.
   */
  CellOccupancyEstimator(double base_occ_prob, double base_empty_prob):
    _base_occ_prob(base_occ_prob), _base_empty_prob(base_empty_prob) {}

  /**
   * Estimates the probaility of cell occupancy according to a strategy defined
   * by subclasses.
   * \param beam A beam bounded with a given cell.
   * \param cell_bnds Bounds of a cell in the context of the world.
   * \param is_occ The probability of a cell to be occupied.
   */
  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) = 0;
protected:

  /// Returns the probability of being occupied.
  double base_occ_prob() { return _base_occ_prob; }

  /// Returns the probability of being empty.
  double base_empty_prob() { return _base_empty_prob; }
private:
  double _base_occ_prob, _base_empty_prob;
};

#endif
