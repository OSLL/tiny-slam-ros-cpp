/**
 * \file
 * \brief Defines some structures related to data obtained from sensors.
 * There are structures ScanPoint and TransformedLaserScan.
 */

#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <vector>

/**
 * \brief Contains a point in polar coordinates and a state whether it is
 * occupied.
 */
struct ScanPoint {

  /**
   * Initializes a point with given parameters.
   * Creates an occupied point in (0,0) by default.
   */
  ScanPoint(double rng = 0, double ang = 0, bool is_occ = true):
    range(rng), angle(ang), is_occupied(is_occ) {}

  double range; ///< The range of point in polar.
  double angle; ///< The angle of point in polar (in radians).
  bool is_occupied; ///< True, if this point is occupied and False otherwise.
};

/**
 * \brief Framework internal representation of a laser scan.
 */
struct TransformedLaserScan {
  double d_x, d_y, d_yaw; ///< The odometry delta.

  std::vector<ScanPoint> points; ///< The vector of points on scan.
  double quality; ///< The quality of scan. 0 - low, 1 - fine.
};

#endif
