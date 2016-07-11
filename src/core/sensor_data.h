#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <vector>

struct ScanPoint {
  ScanPoint(double rng = 0, double ang = 0, bool is_occ = true):
    range(rng), angle(ang), is_occupied(is_occ) {}

  double range;
  double angle; // radians
  bool is_occupied;
};

struct TransformedLaserScan {
  // odomenry delta
  double d_x, d_y, d_yaw;

  std::vector<ScanPoint> points;
  double quality; // 0 - low, 1 - fine
};

#endif
