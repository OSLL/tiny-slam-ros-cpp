/**
 * \file
 * \brief Discribes some structures related to Scan
 * There are structures ScanPoint and TransformedLaserScan
 */

#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <vector>

/**
 * \brief Struct that contains point in polar coordinates and information about its occupation
 */
struct ScanPoint {
  /// Constructor with parameters. As default creates occepied point in (0,0)
  ScanPoint(double rng = 0, double ang = 0, bool is_occ = true):
    range(rng), angle(ang), is_occupied(is_occ) {}

  double range; ///< range of point in polar
  double angle; ///< angle of point in polar in radians
  bool is_occupied; ///< True, if this point is occupied and False otherwise
};
/**
 * \brief Struct that contains information about all points in scan and odometry delta
 */
struct TransformedLaserScan {
  double d_x, d_y, d_yaw; ///< Odometry delta

  std::vector<ScanPoint> points; ///< Vector of points on scan
  double quality; /// Quality of scan. 0 - low, 1 - fine
};

#endif
