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

  void invalidate(){
    prob_occ = std::numeric_limits<double>::quiet_NaN();
    estimation_quality = std::numeric_limits<double>::quiet_NaN();
  }

  bool is_valid() const {
    return !std::isnan(prob_occ) && !std::isnan(estimation_quality);
  }
};
/**
 * The representation of a laser beam bounded by an obstacle.
 */
struct Beam {
  double x_st, y_st; ///< Coordinates of a start of the beam.
  double x_end, y_end; ///< Coordinates where the beam reaches an obstacle.

  //methods
  bool touches_rect(const Rectangle& bnds) const {
    return equal(x_st, x_end, bnds.left) || equal(y_st, y_end, bnds.bot) ||
           equal(x_st, x_end, bnds.right) || equal(y_st, y_end, bnds.top);
  }

  Beam operator- () const {
    return (*this).revert();
  }



  bool intersects(const Rectangle& bnds) const {
    bool is_inside_rect = bnds.is_inside(x_st, y_st) ||
                          bnds.is_inside(x_end, y_end);
    if (is_inside_rect) {
      return true;
    }

    bool result = false;
    Ray ray(x_st, y_st, x_end - x_st, y_end - y_st);
    Intersections intrs = ray.find_intersections(bnds);
    for (auto intersection : intrs) {
      result |= bnds.is_on_border(intersection.x, intersection.y);
    }
    return result;
  }

  bool encounters(const Rectangle& bnds) const {
    if(!bnds.is_on_border(x_end, y_end) || touches_rect(bnds)) {
      return false;
    }
    if (equal(x_end, bnds.left) || equal(x_end, bnds.right)) {
      return are_strictly_ordered(x_st, x_end, bnds.center_x());
    }
    if (equal(y_end, bnds.bot) || equal(y_end, bnds.top)) {
      return are_strictly_ordered(y_st, y_end, bnds.center_y());
    }
    return false;
  }

  bool reaches_border_passing_through(const Rectangle& bnds) const {
    return bnds.is_on_border(x_end, y_end) && intersects(bnds);
  }
  Beam revert() const {
    Beam result(*this);
    std::swap(result.x_st, result.x_end);
    std::swap(result.y_st, result.y_end);
    return result;
  }

  Beam move_start_out(const Rectangle& bnds) const {
    Beam result(*this);

    bool beam_ends_inside = !bnds.is_on_border(x_st, y_st) &&
                            !bnds.is_on_border(x_end,y_end) &&
                            bnds.is_inside(x_end, y_end);
    bool beam_starts_inside = bnds.is_inside(result.x_st, result.y_st);
    while (beam_ends_inside && beam_starts_inside) {
      result.x_st = result.x_st - (result.x_end - result.x_st);
      result.y_st = result.y_st - (result.y_end - result.y_st);
      beam_starts_inside = bnds.is_inside(result.x_st, result.y_st);
    }

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
