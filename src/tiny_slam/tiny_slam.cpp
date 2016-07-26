/*!
 * \file
 * \brief Main file with the entry point
 *
 * There are main and functions which parse initialization launch file.
 * Moreover there is a declaration of one class (PoseScanMatcherObserver is inherited from GridScanMatcherObserver)
 * This file includes such files as: area_occupancy_estimator.h, const_occupancy_estimator.h, grid_cell_strategy.h, tiny_fascade.h,tiny_world.h, tiny_grid_cells.h
 */

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

#include <nav_msgs/OccupancyGrid.h>

//#define RVIZ_DEBUG 1

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/rviz_grid_viewer.h"
#include "../ros/utils.h"
#include "../core/maps/area_occupancy_estimator.h"
#include "../core/maps/const_occupancy_estimator.h"
#include "../core/maps/grid_cell_strategy.h"
#include "tiny_fascade.h"
#include "tiny_world.h"
#include "tiny_grid_cells.h"

/*!
 * \brief Derived class from GridScanMatcherObserver to publish robot pose
 *
 * This class provides to functions to publish robot pose in ros topic
 */
class PoseScanMatcherObserver : public GridScanMatcherObserver {
public:

  /*!
   * Functions publish with child frame sm_curr_pose and base frame odom_combined
   * \param[in] pose - robot location in space
   */
  virtual void on_scan_test(const RobotState &pose,
                            const TransformedLaserScan &scan,
                            double score) override {
    publish_transform("sm_curr_pose", pose);
  }
  /*!
   * Functions publish with child frame sm_best_pose and base frame odom_combined
   * \param[in] pose - robot location in space
   */
  virtual void on_pose_update(const RobotState &pose,
                              const TransformedLaserScan &scan,
                              double score) override {
    publish_transform("sm_best_pose", pose);
  }
private:
    /*!
     * private function which publish the robot state
     * \param[in] frame_id - the child frame
     *  \param[in] p - robot location in space
     */
    void publish_transform(const std::string& frame_id, const RobotState& p) {
      publish_2D_transform(frame_id, "odom_combined", p.x, p.y, p.theta);
    }
};


/*!
 * Functions sets the cell factory in case of what parameters came from launch file
 * \param[in] params - values from launch files
 *  \return The pointer (shared) on created factory of grid cells
 */
std::shared_ptr<GridCellFactory> init_cell_factory(TinyWorldParams &params) {
  std::string cell_type;
  ros::param::param<std::string>("~cell_type", cell_type, "avg");

  if (cell_type == "base") {
    params.localized_scan_quality = 0.2;
    params.raw_scan_quality = 0.1;
    return std::shared_ptr<GridCellFactory>{new TinyBaseCellFactory()};
  } else if (cell_type == "avg") {
    params.localized_scan_quality = 0.9;
    params.raw_scan_quality = 0.6;
    return std::shared_ptr<GridCellFactory>{new TinyAvgCellFactory()};
  } else {
    std::cerr << "Unknown cell type: " << cell_type << std::endl;
    std::exit(-1);
  }
}
/*!
 * Functions sets the estimator in case of what parameters came from launch file
 * \param[in] params - values from launch files
 * \return The pointer (shared) on created estimator of map cost
 */
std::shared_ptr<CellOccupancyEstimator> init_occ_estimator() {
  double occ_prob, empty_prob;
  ros::param::param<double>("~base_occupied_prob", occ_prob, 0.95);
  ros::param::param<double>("~base_empty_prob", empty_prob, 0.01);

  using OccEstPtr = std::shared_ptr<CellOccupancyEstimator>;
  std::string est_type;
  ros::param::param<std::string>("~occupancy_estimator", est_type, "const");

  if (est_type == "const") {
    return OccEstPtr{new ConstOccupancyEstimator(occ_prob, empty_prob)};
  } else if (est_type == "area") {
    return OccEstPtr{new AreaOccupancyEstimator(occ_prob, empty_prob)};
  } else {
    std::cerr << "Unknown estimator type: " << est_type << std::endl;
    std::exit(-1);
  }
}

/*!
 * Function returns the launch file configured parameter how to deal with exceeding values
 */
bool init_skip_exceeding_lsr() {
  bool param_value;
  ros::param::param<bool>("~skip_exceeding_lsr_vals", param_value, false);
  return param_value;
}

/*!
 * The entry point where it creates an environment world based on coming parameters.
 * After that it is subscribed on ros topic. Than the viewer/map-builder is created and it is started.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "tinySLAM");

  ros::NodeHandle nh;
  TinyWorldParams params;
  std::shared_ptr<ScanCostEstimator> cost_est{new TinyScanCostEstimator()};
  std::shared_ptr<GridCellStrategy> gcs{new GridCellStrategy{
    init_cell_factory(params), cost_est, init_occ_estimator()}};
  std::shared_ptr<TinySlamFascade> slam{new TinySlamFascade(gcs,
    params, init_skip_exceeding_lsr())};

  TopicWithTransform<sensor_msgs::LaserScan> scan_observer(nh,
      "laser_scan", "odom_combined");
  scan_observer.subscribe(slam);

  std::shared_ptr<RvizGridViewer> viewer(
    new RvizGridViewer(nh.advertise<nav_msgs::OccupancyGrid>("/map", 5)));
  slam->set_viewer(viewer);

#ifdef RVIZ_DEBUG
  std::shared_ptr<PoseScanMatcherObserver> obs(new PoseScanMatcherObserver);
  slam->add_scan_matcher_observer(obs);
#endif
  ros::spin();
}
