#ifndef __TINY_SLAM_FASCADE_H
#define __TINY_SLAM_FASCADE_H

#include <memory>

#include "../ros/laser_scan_observer.h"
#include "../ros/rviz_grid_viewer.h"
#include "../core/maps/grid_cell_strategy.h"

#include "tiny_world.h"

/*!
 * \brief Derived class from LaserScanObserver to manage all world
 *
 * Handles data came from laser scanner and locates views if he is set.
 * Presents a pattern fascade for all algorithm "tiny slam"
 *
 */
class TinySlamFascade : public LaserScanObserver {
private:
  using ScanMatcherObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public: // methods
  // TODO: copy ctor, move ctor, dtor

  /*!
   * Parameterized constructor sets all data members
   * \param[in] gcs           - Grid Cell Strategy - configuration of cells in map
   * \param[in] params        - configuration of laser scan (sets the quality value)
   * \param[in] skip_max_vals - boolean variable - flag shows how are high values in messages will be handeled
   */
  TinySlamFascade(std::shared_ptr<GridCellStrategy> gcs,
                  const TinyWorldParams &params,
                  bool skip_max_vals):
    LaserScanObserver(skip_max_vals), _world(new TinyWorld(gcs, params)) {}

  /*!
   * Setter for data member "viewer"
   * \param[in] viewer - new value of data member viewer
   */
  void set_viewer(std::shared_ptr<RvizGridViewer> viewer) {
    _viewer = viewer;
  }

  /*!
   * Updates the building map and robot pose using scan data
   * 
   * It shifts the robot pose on scanner odnometry parameters (\f$\Delta x, \Delta y, \Delta\theta\f$) and update the map using abilities presented in "world"
   * \param[in,out] scan - data from laser scanner, the view that robot has seen just a moment
   * (it is out variable as during this method there makes a decision about its value robust)
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
   * Modifier of data member "world", which adds a scanner matcher
   * \param[in] obs - new scanner matcher
   */
  void add_scan_matcher_observer(ScanMatcherObsPtr obs) {
    _world->scan_matcher()->subscribe(obs);
  }
  /*!
   * Modifier of data member "world", which removes a scanner matcher
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
