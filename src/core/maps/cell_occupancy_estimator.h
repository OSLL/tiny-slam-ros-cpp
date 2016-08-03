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
 * Contains information about probability of occupancy and quality of
 * estimation for 2D point in cartesian coordinates(x,y).
 */
struct Occupancy {
  double prob_occ;
  double estimation_quality;
  double x, y; ///< Estimated obstacle center

  /**
   * Initializes probability of occupation and quality of estimation.
   * \param A probability of occupation of current cell.
   * \param quality A quality of estimation for current cell.
   */
  Occupancy(double prob, double quality) :
    prob_occ(prob), estimation_quality(quality) {}

  /**
   * Initializes probability of occupation and quality of estimation for a
   * point in cartesian coordinates (x,y).
   * \param prob A probability of occupation of current cell.
   * \param quality A quality of estimation for current cell.
   * \param x,y Coordinates of an occupied cell
   */
  Occupancy(double prob, double quality, double curr_x, double curr_y) :
    prob_occ(prob), estimation_quality(quality), x(curr_x), y(curr_y) {}

  bool operator==(const Occupancy &that) {
    return EQ_DOUBLE(prob_occ, that.prob_occ) &&
           EQ_DOUBLE(estimation_quality, that.estimation_quality);
  }
};
/**
 * The representation of a laser beam bounded by an obstacle.
 */
struct Beam {
  double x_st, y_st; ///< Coordinates of start of beam.
  double x_end, y_end; ///< Coordinates where beam reaches an obstacle.
};

/**
 * Estimates the occupancy of cell according to a chosen strategy.
 */
class CellOccupancyEstimator {
public:

  /**
   * Initializes an estimator with probability of cell to be occupied and empty.
   * \param base_occ_prob Initial probability of a cell to be occupied.
   * \param base_empty_prob Initial probability of a cell to be empty.
   */
  CellOccupancyEstimator(double base_occ_prob, double base_empty_prob):
    _base_occ_prob(base_occ_prob), _base_empty_prob(base_empty_prob) {}

  /**
   * Estimates the probaility of cell occupancy according to a chosen strategy.
   * \param beam A beam bounded with a given cell.
   * \param cell_bnds Bounds of a cell in the context of the world.
   * \param is_occ The probability of a cell to be occupied.
   */
  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) = 0;
protected:

  /// Returns the occupancy probability.
  double base_occ_prob() { return _base_occ_prob; }

  /// Returns the empty probability.
  double base_empty_prob() { return _base_empty_prob; }
private:
  double _base_occ_prob, _base_empty_prob;
};

#endif
