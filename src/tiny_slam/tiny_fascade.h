#ifndef __TINY_SLAM_FASCADE_H
#define __TINY_SLAM_FASCADE_H

#include <memory>

#include "../ros/laser_scan_observer.h"
#include "../ros/rviz_grid_viewer.h"
#include "../core/maps/grid_cell_strategy.h"

#include "tiny_world.h"

/*!
 * \brief The TinySLAM access point. SLAM users are supposed to work with it.
 *
 * Responsibilities:\n
 *   Handles data came from a scanner and locates a viewer.\n
 *   Connects internal frameworks components to a single tinySLAM method
 *   (the fascade pattern is applied).
 *
 */
class TinySlamFascade : public LaserScanObserver {
private:
  using ScanMatcherObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public: // methods
  // TODO: copy ctor, move ctor, dtor
  /*!
   * Initializes the tinySLAM method.
   * \param[in] gcs           - a configuration of cells in a map
   *                            (a cell strategy).
   * \param[in] params        - the tinySLAM parameters (see TinyWorldParams).
   * \param[in] skip_max_vals - whether the values that exceed the max one
   *                            specific to the laser scanner should be skipped.
   */
  TinySlamFascade(std::shared_ptr<GridCellStrategy> gcs,
                  const TinyWorldParams &params,
                  const GridMapParams &init_map_params,
                  bool skip_max_vals):
    LaserScanObserver(skip_max_vals),
    _world(new TinyWorld(gcs, params, init_map_params)) {}

  /*!
   * Sets a viewer component that is notified by a pose and map updates.
   * \param[in] viewer - a new value of the data member viewer.
   */
  void set_viewer(std::shared_ptr<RvizGridViewer> viewer) {
    _viewer = viewer;
  }

  /*!
   * Updates the map and the robot pose with scan data.\n
   * The update is done according to a prediction-correction approach.
   * (the odometry is used for a prediction, the laser scan - for a correction).
   * \param[in] scan - data from the robot's scanners (odnometry + laser scan).
   */
  virtual void handle_laser_scan(TransformedLaserScan &scan) {
    _world->update_robot_pose(scan.d_x, scan.d_y, scan.d_yaw);
    _world->handle_observation(scan);

    if (_viewer) {
      _viewer->show_robot_pose(_world->pose());
      _viewer->show_map(_world->map());
    }
  }

  /*!
   * Registers a scan matcher observer.
   * \param[in] obs - a new scanner matcher observer.
   */
  void add_scan_matcher_observer(ScanMatcherObsPtr obs) {
    _world->scan_matcher()->subscribe(obs);
  }

  /*!
   * Unregisters a scan matcher observer.
   * \param[in] obs - the scanner matcher observer to be removed.
   */
  void remove_scan_matcher_observer(ScanMatcherObsPtr obs) {
    _world->scan_matcher()->unsubscribe(obs);
  }

private:
  // Position by IMU, used to calculate delta
  std::unique_ptr<TinyWorld> _world;
  std::shared_ptr<RvizGridViewer> _viewer;
};

#endif
