#ifndef AREA_ESTIMATOR_TEST_H_
#define AREA_ESTIMATOR_TEST_H_

#include "../../src/core/maps/area_occupancy_estimator.h"

struct Ray {
  Ray() : Ray(0, 0, 0, 0) {}
  Ray(double s_x, double s_y, double e_x, double e_y) :
    st_x(s_x), st_y(s_y), end_x(e_x), end_y(e_y) {}
  double st_x, st_y, end_x, end_y;
};

bool test_estimator(const Rectangle &cell, bool is_occ,
                    const Ray &ray, const Occupancy &expected) {
  AreaOccupancyEstimator aoe(0.95, 0.01);
  Beam beam{ray.st_x, ray.st_y, ray.end_x, ray.end_y};
  Occupancy occ = aoe.estimate_occupancy(beam, cell, is_occ);
  if (expected.isNan())
    return occ.isNan(); 

  return occ == expected;
}

Rectangle read_cell(double top, double bot, double left, double right) {
  Rectangle cell;
  cell.top   = top; 
  cell.bot   = bot;
  cell.left  = left;
  cell.right = right;
  return cell;
}

Ray read_ray(double st_x, double st_y, double end_x, double end_y) {
  Ray ray;
  ray.st_x  = st_x; 
  ray.st_y  = st_y;
  ray.end_x = end_x;
  ray.end_y = end_y;
  return ray;
}

#endif
