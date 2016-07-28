#ifndef __TINY_SCAN_COST_ESTIMATOR
#define __TINY_SCAN_COST_ESTIMATOR

#include "../core/grid_scan_matcher.h"

/*!
 * \brief Derived class from ScanCostEstimator to calculate "cost" of one scan
 *
 * This class presents one method to calculate a numeric value of one frame. This method is defined in the article.
 */
class TinyScanCostEstimator : public ScanCostEstimator {
public:

  /*!
   * Function to estimate cost of values from laser scan which are given from every direction from one pose of robot.
   *
   * It is summed up the all probabilities from cells in a map which came as occupied from laser scan.
   * This value then lower then there is more coincidences between scanner data and data from built map.
   *
   * \param[in] pose     - the robot pose in the space
   * \param[in] scan     - array of points given from the laser scanner when robot was located in one place
   * \param[in] map      - the environment map built on the previous steps
   * \param[in] min_cost - the cost value which presented as a limit after that there will be no reason to calculate this scan
   * \return the value cost of current scan
   */
  virtual double estimate_scan_cost(const RobotState &pose,
                                    const TransformedLaserScan &scan,
                                    GridMap &map,
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
