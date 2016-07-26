/*!
 * \file
 * \brief Description of class file (TinyScanCostEstimator is inherited from ScanCostEstimator)
 *
 * This file includes one class TinyScanCostEstimator presented a realization of virtual methods of ScanCostEstimator class: the method to calculate the cost of scanner data based on existing built map.
 */

#ifndef __TINY_SCAN_COST_ESTIMATOR
#define __TINY_SCAN_COST_ESTIMATOR

#include "../core/grid_scan_matcher.h"

/*!
 * \brief Derived class from ScanCostEstimator to calculate "cost" of one scan
 *
 * This class presents one method to calculate a numeric value of one frame.
 * There is one overriding function were all logic presented.
 */
class TinyScanCostEstimator : public ScanCostEstimator {
public:

  /*!
   * Function to calculate cost of values from laser scan which are given from every direction from one pose of robot.
   *
   * It is calculated a specific value "cost" which presented the sum all probabilities of cells which contain the points got from scanner.
   * For every point come from laser scanner it is calculating its cell coordinates - coordinates of map grid cell where the current point locates.
   * (if this cell do not exist (gets out of range) - this point data gets the max_cost_value=1)
   * This this cell locates on the map, then it is returned a probability value of this cell and inversion of this value (\f$1-p_i\f$) is added to the cost of all scanner frame
   * This procedure stops when the cost value becomes greater than min_cost value got as a parameter.
   *
   * \param[in] pose     - the robot pose in the space
   * \param[in] scan     - array of points given from the laser scanner when robot was located in one place
   * \param[in] map      - the environment map built on the previous steps
   * \param[in] min_cost - the cost value which presented as a limit after that there will be no reason to calculate this scan
   * \return the value cost of current scan
   */
  virtual double estimate_scan_cost(const RobotState &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost) override {
    double cost = 0;
    for (const auto &sp : scan.points) {
      if (!sp.is_occupied) {
        continue;
      }
      // move to world frame assume sensor coords (0,0)
      double x_world = pose.x + sp.range * std::cos(sp.angle+pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle+pose.theta);

      DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world);
      if (!map.has_cell(cell_coord)) {
        cost += 1.0;
        continue;
      }
      double cell_value = map.cell_value(cell_coord);
      cost += 1.0 - cell_value;
      if (min_cost < cost) {
        break;
      }
    }
    return cost;
  }

};

#endif
