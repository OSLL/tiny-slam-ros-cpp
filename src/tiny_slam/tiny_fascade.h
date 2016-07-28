#ifndef __TINY_SLAM_FASCADE_H
#define __TINY_SLAM_FASCADE_H

#include <memory>

#include "../ros/laser_scan_observer.h"
#include "../ros/rviz_grid_viewer.h"
#include "../core/maps/grid_cell_strategy.h"

#include "tiny_world.h"

/*!
 * \brief TinySLAM access point. SLAM users are supposed to work with it.
 *
 * Responsibilities:
 * Handles data came from laser scanner and locates viewer.
 * Connects internal frameworks components to a single tinySLAM method (the fascade pattern is applied)
 *
 */
class TinySlamFascade : public LaserScanObserver {
private:
  using ScanMatcherObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public: // methods
  // TODO: copy ctor, move ctor, dtor
  /*!
   * Initializes the tinySLAM method
   * \param[in] gcs           - Grid Cell Strategy - configuration of cells in map
   * \param[in] params        - configuration of laser scan
   * \param[in] skip_max_vals - boolean variable - flag shows how are the values that exceed the max one specific to laser scanner are handled
   */
  TinySlamFascade(std::shared_ptr<GridCellStrategy> gcs,
                  const TinyWorldParams &params,
                  bool skip_max_vals):
    LaserScanObserver(skip_max_vals), _world(new TinyWorld(gcs, params)) {}

  /*!
   * Sets a viewer component that is notified by pose and map updates.
   * \param[in] viewer - new value of data member viewer
   */
  void set_viewer(std::shared_ptr<RvizGridViewer> viewer) {
    _viewer = viewer;
  }

  /*!
   * Updates the map and robot pose with scan data
   * 
   * The update is done by prediction-correction approach (odometry is used for prediction, laser scan - for correction)
   * \param[in] scan - data from laser scanner, the view that robot has seen just a moment
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
   * Registers a scan matcher observer
   * \param[in] obs - new scanner matcher
   */
  void add_scan_matcher_observer(ScanMatcherObsPtr obs) {
    _world->scan_matcher()->subscribe(obs);
  }
  /*!
   * Unregisters a scan matcher observer
   * \param[in] obs - the scanner matcher
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
