/*!
 * \file
 * \brief The ROS node implementation that provides the tinySLAM method.
 *
 * There are an entry point and functions which parse the initialization file.\n
 * There is also a declaration of one class
 * (PoseScanMatcherObserver is inherited from GridScanMatcherObserver).
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
 * \brief Derived class from GridScanMatcherObserver to publish the robot pose.
 *
 * This class provides functions to publish the robot pose in a ROS topic.
 */
class PoseScanMatcherObserver : public GridScanMatcherObserver {
public:

  /*!
   * Publishes the robot pose tested by the scan matcher at the moment.
   * \param[in] pose - the robot location in the space.
   */
  virtual void on_scan_test(const RobotState &pose,
                            const TransformedLaserScan &scan,
                            double score) override {
    publish_transform("sm_curr_pose", pose);
  }
  /*!
   * Publishes the best found robot pose.
   * \param[in] pose - the robot location in the space.
   */
  virtual void on_pose_update(const RobotState &pose,
                              const TransformedLaserScan &scan,
                              double score) override {
    publish_transform("sm_best_pose", pose);
  }
private:
    void publish_transform(const std::string& frame_id, const RobotState& p) {
      publish_2D_transform(frame_id, "odom_combined", p.x, p.y, p.theta);
    }
};

/*!
 * Determines the cell factory based on parameters came from a launch file.
 * \param[in] params - values from the launch file.
 * \return The pointer (shared) to a created factory of grid cells.
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
 * Determines the estimator based on parameters came from a launch file.
 * \param[in] params - values from a launch file.
 * \return The pointer (shared) to a created estimator of a map cost.
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
 * Returns how to deal with exceeding values based on parameters came
 * from a launch file.
 */
bool init_skip_exceeding_lsr() {
  bool param_value;
  ros::param::param<bool>("~skip_exceeding_lsr_vals", param_value, false);
  return param_value;
}

/**
 * Initializes constants for scan matcher
 * \return The structure contains requied paramteres
 */
TinyWorldParams init_common_world_params() {
  double sig_XY, sig_T, width;
  int lim_bad, lim_totl;
  ros::param::param<double>("~scmtch_sigma_XY_MonteCarlo", sig_XY, 0.2);
  ros::param::param<double>("~scmtch_sigma_theta_MonteCarlo", sig_T, 0.1);
  ros::param::param<int>("~scmtch_limit_of_bad_attempts", lim_bad, 20);
  ros::param::param<int>("~scmtch_limit_of_total_attempts", lim_totl, 100);
  ros::param::param<double>("~hole_width", width,1.5);

  return TinyWorldParams(sig_XY, sig_T, lim_bad, lim_totl, width);
}

/**
 * Initializes constants for map
 * \return The structure contains requied paramteres
 */
GridMapParams init_grid_map_params() {
  GridMapParams params;
  ros::param::param<double>("~map_height_in_meters", params.height, 20);
  ros::param::param<double>("~map_width_in_meters", params.width, 20);
  ros::param::param<double>("~map_meters_per_cell", params.meters_per_cell,
                                                                         0.1);
  return params;
}

/**
 * Initializes constants for ros utils
 * \return Requied parameters
 */
void init_constants_for_ros(double &ros_tf_buffer_size,
                            double &ros_map_rate,
                            int &ros_filter_queue,
                            int &ros_subscr_queue) {
  ros::param::param<double>("~ros_tf_buffer_duration",ros_tf_buffer_size,5.0);
  ros::param::param<double>("~ros_rviz_map_publishing_rate", ros_map_rate, 5.0);
  ros::param::param<int>("~ros_filter_queue_size",ros_filter_queue,1000);
  ros::param::param<int>("~ros_subscribers_queue_size",ros_subscr_queue,1000);
}

/*!
 * The entry point: creates an environment world and the main node "tiny slam".
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "tinySLAM");

  ros::NodeHandle nh;
  TinyWorldParams params = init_common_world_params();
  GridMapParams grid_map_params = init_grid_map_params();
  std::shared_ptr<ScanCostEstimator> cost_est{new TinyScanCostEstimator()};
  std::shared_ptr<GridCellStrategy> gcs{new GridCellStrategy{
    init_cell_factory(params), cost_est, init_occ_estimator()}};
  std::shared_ptr<TinySlamFascade> slam{new TinySlamFascade(gcs,
    params, grid_map_params, init_skip_exceeding_lsr())};

  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  TopicWithTransform<sensor_msgs::LaserScan> scan_observer(nh,
    "laser_scan", "odom_combined", ros_tf_buffer_size,
    ros_filter_queue, ros_subscr_queue);
  scan_observer.subscribe(slam);

  std::shared_ptr<RvizGridViewer> viewer(
    new RvizGridViewer(nh.advertise<nav_msgs::OccupancyGrid>("/map", 5),
                       ros_map_publishing_rate));
  slam->set_viewer(viewer);

#ifdef RVIZ_DEBUG
  std::shared_ptr<PoseScanMatcherObserver> obs(new PoseScanMatcherObserver);
  slam->add_scan_matcher_observer(obs);
#endif
  ros::spin();
}
