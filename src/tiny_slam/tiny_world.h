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

struct TinyWorldParams {
  double localized_scan_quality, raw_scan_quality;
  const double SIG_XY;
  const double SIG_TH;
  const unsigned BAD_LMT;
  const unsigned TOT_LMT;
  const double HOLE_WIDTH;

  TinyWorldParams(double sig_XY, double sig_T, unsigned lim_bad,
                  unsigned lim_totl, double hole_width) :
    SIG_XY(sig_XY), SIG_TH(sig_T), BAD_LMT(lim_bad), TOT_LMT(lim_totl),
    HOLE_WIDTH(hole_width) {}
};

class TinyWorld : public LaserScanGridWorld {
public:
  using Point = DiscretePoint2D;
public:

  TinyWorld(std::shared_ptr<GridCellStrategy> gcs,
            const TinyWorldParams &params,
            const GridMapParams &init_map_params) :
    LaserScanGridWorld(gcs, init_map_params), _gcs(gcs), _params(params),
    _scan_matcher(new TinyScanMatcher(_gcs->cost_est(),
                                      params.BAD_LMT, params.TOT_LMT,
                                      params.SIG_XY, params.SIG_TH)) {}

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

  virtual void handle_scan_point(GridMap &map,
                                 double laser_x, double laser_y,
                                 double beam_end_x, double beam_end_y,
                                 bool is_occ, double quality) override {
    Beam beam{laser_x, laser_y, beam_end_x, beam_end_y};
    Point robot_pt = map.world_to_cell(laser_x, laser_y);
    Point obst_pt = map.world_to_cell(beam_end_x, beam_end_y);

    double obst_dist_sq = robot_pt.dist_sq(obst_pt);
    std::vector<Point> pts = DiscreteLine2D(robot_pt, obst_pt).points();
    double hole_dist_sq = std::pow(_params.HOLE_WIDTH / map.cell_scale(), 2);

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
  std::shared_ptr<GridCellStrategy> _gcs;
  const TinyWorldParams _params;
  std::shared_ptr<TinyScanMatcher> _scan_matcher;
};

#endif
