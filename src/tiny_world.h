/*
Copyright (c) 2016 JetBrains Research, Mobile Robot Algorithms Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy of this 
software and associated documentation files (the "Software"), to deal in the Software 
without restriction, including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __TINY_WORLD_H
#define __TINY_WORLD_H

#include <cmath>

#include "state_data.h"
#include "sensor_data.h"
#include "grid_map.h"
#include "geometry_utils.h"

#include "tiny_scan_matcher.h"

//#define USE_BASE_CELL 1

class BaseTinyCell {
public:
  BaseTinyCell(): _prob(0.5) {}
  double value() const { return _prob; }
  void set_value(double value, double quality) {
    _prob = (1.0 - quality) * _prob + quality * value;
  }
private:
  double _prob;
};

class AvgTinyCell {
public:
  AvgTinyCell(): _cnt(0), _n(0) {}
  double value() const //{ return _prob; }
  { return _n == 0 ? -1 : _cnt / _n; }
  void set_value(double value, double quality) {
    _n+=1;
    _cnt += 0.5 + (value - 0.5) * quality;
  }
private:
  double _cnt, _n;
};

#ifndef USE_BASE_CELL
  using TinyCell = AvgTinyCell;
#else
  using TinyCell = BaseTinyCell;
#endif

class TinyWorld : public World<TransformedLaserScan, GridMap<TinyCell>> {
private: // internal params
#ifndef USE_BASE_CELL
  // AvgTinyCell model params
  // Scan matcher
  const double SIG_XY = 0.2;
  const double SIG_TH = 0.1;
  const double BAD_LMT = 20;
  const double TOT_LMT = 100;
  // Map update
  const double LOCALIZED_SCAN_Q = 0.6;
  const double RAW_SCAN_Q = 0.9;

  const double OBST_PROB = 0.95;
  const double NONOBST_PROB = 0.01;

  const double HOLE_WIDTH = 1.5; // m
  const double HOLE_BASE_PROB = 0.75;
#else
  // BaseTinyCell params
  // Scan matcher
  const double SIG_XY = 0.01;
  const double SIG_TH = 0.007;
  const double BAD_LMT = 10;
  const double TOT_LMT = BAD_LMT * 5;
  // Map update
  const double RAW_SCAN_Q = 0.2;
  const double LOCALIZED_SCAN_Q = RAW_SCAN_Q / 10;

  const double OBST_PROB = 0.95;
  const double NONOBST_PROB = 0.01;

  const double HOLE_WIDTH = 1.5;
  const double HOLE_BASE_PROB = OBST_PROB;
#endif

public:
  using MapType = GridMap<TinyCell>;
  using Point = DiscretePoint2D;
public:

  virtual void handle_observation(TransformedLaserScan &scan) override {
    RobotState pose_delta;
    TinyScanMatcher<TinyCell> scan_matcher(BAD_LMT, TOT_LMT, SIG_XY, SIG_TH);
    scan_matcher.process_scan(pose(), scan, map(), pose_delta);
    update_robot_pose(pose_delta.x, pose_delta.y, pose_delta.theta);

    bool pose_was_fixed = pose_delta.x || pose_delta.y || pose_delta.theta;
    scan.quality = pose_was_fixed ? LOCALIZED_SCAN_Q : RAW_SCAN_Q;

    const RobotState& pose = World<TransformedLaserScan, MapType>::pose();
    for (const auto &sp : scan.points) {
      // move to world frame assuming sensor is in robots' (0,0)
      double x_world = pose.x + sp.range * std::cos(sp.angle + pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle + pose.theta);

      Point robot_pos = _map.world_to_cell(pose.x, pose.y);
      Point obst_pos = _map.world_to_cell(x_world, y_world);

      handle_scan_point(map(), robot_pos, obst_pos,
                        sp.is_occupied, scan.quality);
    }
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

private:

  void handle_scan_point(MapType &map, Point &robot_pt, Point &obst_pt,
                         bool is_occ, double quality) {
    double obst_dist_sq = robot_pt.dist_sq(obst_pt);
    std::vector<Point> pts = DiscreteLine2D(robot_pt, obst_pt).points();
    double hole_dist_sq = std::pow(HOLE_WIDTH / map.cell_scale(), 2);

    map.update_cell(pts.back(), is_occ ? OBST_PROB : NONOBST_PROB, quality);
    pts.pop_back();
    for (const auto &pt : pts) {
      double cell_value = NONOBST_PROB;
      double dist_sq = robot_pt.dist_sq(pt);
      double diff = obst_dist_sq - dist_sq;
      if (diff < hole_dist_sq && hole_dist_sq < obst_dist_sq) {
        cell_value += HOLE_BASE_PROB * (1.0 - diff / hole_dist_sq);
      }
      map.update_cell(pt, cell_value, quality);
    }
  }

private: // fields
  MapType _map;

};

#endif
