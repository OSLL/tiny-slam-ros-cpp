/*!
 * \brief Description of class file (TinyWorld is inherited from LaserScanGridWorld)
 *
 * There are structure TinyWorldParams which has two parameters of scan quality and class TinyWorld
 * there are included such files as: state_data.h, sensor_data.h, laser_scan_grid_world.h, grid_cell_strategy.h, tiny_grid_cells.h, tiny_scan_matcher.h
 */

#ifndef __TINY_WORLD_H
#define __TINY_WORLD_H

#include <iostream>
#include <cmath>
#include <memory>

#include "../core/state_data.h"
#include "../core/sensor_data.h"
#include "../core/laser_scan_grid_world.h"
#include "../core/maps/grid_cell_strategy.h"

#include "tiny_grid_cells.h"
#include "tiny_scan_matcher.h"

/*!
 * \brief Structure-storage of two different quality of laser scanner
 */
struct TinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
};

/*!
 * \brief
 */
class TinyWorld : public LaserScanGridWorld {
private: // internal params
  // Scan matcher
  const double SIG_XY = 0.2; ///< data member \f$\sigma_{x,y}\f$
  const double SIG_TH = 0.1; ///< data member \f$\sigma_{\theta}\f$
  const double BAD_LMT = 20; ///< amount of bad steps for monte-carlo choice better robot position
  const double TOT_LMT = BAD_LMT * 5; ///< maximum amount of steps are able to be done

  const double HOLE_WIDTH = 1.5; ///< the total width
public:
  using Point = DiscretePoint2D;
public:

  /*!
   * Parameterized constructor sets all data members
   */
  TinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            const TinyWorldParams &params) :
    LaserScanGridWorld(gcs), _gcs(gcs), _params(params),
    _scan_matcher(new TinyScanMatcher(_gcs->cost_est(),
                                      BAD_LMT, TOT_LMT,
                                      SIG_XY, SIG_TH)) {}

  /*!
   * Function updates robot pose and map
   * \param[in] scan - data from laser scanner
   */
  virtual void handle_observation(TransformedLaserScan &scan) override {
    RobotState pose_delta;
    _scan_matcher->reset_state();
    _scan_matcher->process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta.x, pose_delta.y, pose_delta.theta);

    bool pose_was_fixed = pose_delta.x || pose_delta.y || pose_delta.theta;
    auto factory = _gcs->cell_factory();
    scan.quality = pose_was_fixed ? _params.localized_scan_quality :
                                    _params.raw_scan_quality;
    LaserScanGridWorld::handle_observation(scan);
  }

  /*!
   * Function estimates one point from scanner on its occupancy
   * \param[in] map - all built map
   *  \param[in] laser_x,laser_y - the beginning coordinates of the laser ray
   *   \param[in] beam_end_x, beam_end_y - the ending coordinates of the laser ray
   *    \param[in] is_occ - the parameter which shows is this cell occupied or not
   *     \param[in] quality - the quality of laser scanner
   */
  virtual void handle_scan_point(GridMap &map,
                                 double laser_x, double laser_y,
                                 double beam_end_x, double beam_end_y,
                                 bool is_occ, double quality) override {
    Beam beam{laser_x, laser_y, beam_end_x, beam_end_y};
    Point robot_pt = map.world_to_cell(laser_x, laser_y);
    Point obst_pt = map.world_to_cell(beam_end_x, beam_end_y);

    double obst_dist_sq = robot_pt.dist_sq(obst_pt);
    std::vector<Point> pts = DiscreteLine2D(robot_pt, obst_pt).points();
    double hole_dist_sq = std::pow(HOLE_WIDTH / map.cell_scale(), 2);

    auto occ_est = _gcs->occupancy_est();
    Occupancy beam_end_occ = occ_est->estimate_occupancy(beam,
        map.world_cell_bounds(pts.back()), is_occ);
    map.update_cell(pts.back(), beam_end_occ, quality);
    pts.pop_back();

    for (const auto &pt : pts) {
      Occupancy empty_cell_value = occ_est->estimate_occupancy(beam,
          map.world_cell_bounds(pts.back()), false);
      // wall blur
      double dist_sq = pt.dist_sq(obst_pt);
      if (dist_sq < hole_dist_sq && hole_dist_sq < obst_dist_sq && is_occ) {
        empty_cell_value.prob_occ +=
          (1.0 - dist_sq / hole_dist_sq) * beam_end_occ.prob_occ;
      }

      map.update_cell(pt, empty_cell_value, quality);
    }
  }

  std::shared_ptr<GridScanMatcher> scan_matcher() {
    return _scan_matcher;
  }
private:
  std::shared_ptr<GridCellStrategy> _gcs; ///< data member with initial parameters how to calculate the probability of cell occupancy
  const TinyWorldParams _params; ///< data member contains the quality of laser scanner
  std::shared_ptr<TinyScanMatcher> _scan_matcher; ///< data member provided to calculate the optimal robot position
};

#endif
