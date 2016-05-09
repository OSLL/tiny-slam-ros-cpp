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

#ifndef __TINY_SCAN_MATCHER_H
#define __TINY_SCAN_MATCHER_H

#include <random>

#include "sensor_data.h"
#include "state_data.h"

template <typename CellT>
class TinyScanMatcher {
public:
  TinyScanMatcher(unsigned bad_iter, unsigned max_iter,
                  double sigma_coord, double sigma_angle):
    _failed_tries_limit(bad_iter), _total_tries_limit(max_iter),
    _sigma_coord(sigma_coord), _sigma_angle(sigma_angle) {}

  double process_scan(const RobotState &init_pose,
                      const TransformedLaserScan &scan,
                      const GridMap<CellT> &map,
                      RobotState &pose_delta) {

    unsigned failed_tries = 0, total_tries = 0;

    RobotState optimal_pose = init_pose;
    double min_scan_cost = estimate_scan_cost(optimal_pose, scan, map,
                                            std::numeric_limits<double>::max());

    while (failed_tries < _failed_tries_limit &&
           total_tries < _total_tries_limit) {
      total_tries++;

      RobotState sampled_pose = optimal_pose;
      sample_pose(sampled_pose);
      double sampled_scan_cost = estimate_scan_cost(sampled_pose, scan,
                                                    map, min_scan_cost);
      if (min_scan_cost <= sampled_scan_cost) {
        failed_tries++;
        continue;
      }
      min_scan_cost = sampled_scan_cost;
      optimal_pose = sampled_pose;

      if (_failed_tries_limit / 3 < failed_tries) {
        _sigma_coord *= 0.5;
        _sigma_angle *= 0.5;
        failed_tries = 0;
      }
    }

    pose_delta.x = optimal_pose.x - init_pose.x;
    pose_delta.y = optimal_pose.y - init_pose.y;
    pose_delta.theta = optimal_pose.theta - init_pose.theta;
    return min_scan_cost;
  }

private:
  double estimate_scan_cost(const RobotState &pose,
                            const TransformedLaserScan &scan,
                            const GridMap<CellT> &map,
                            double min_cost) {
    double cost = 0;
    for (const auto &sp : scan.points) {
      if (!sp.is_occupied) {
        continue;
      }
      // move to world frame assume sensor coords (0,0)
      double x_world = pose.x + sp.range * std::cos(sp.angle+pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle+pose.theta);

      double cell_value = map.cell_value(map.world_to_cell(x_world, y_world));
      cost += 1.0 - cell_value;
      if (min_cost < cost) {
        break;
      }
    }
    return cost;
  }

  void sample_pose(RobotState &base_pose) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_coord(0.0, _sigma_coord);
    std::normal_distribution<> d_angle(0.0, _sigma_angle);

    base_pose.x += d_coord(gen);
    base_pose.y += d_coord(gen);
    base_pose.theta += d_angle(gen);
  }

private:
  unsigned _failed_tries_limit;
  unsigned _total_tries_limit;
  double _sigma_coord, _sigma_angle;
};

#endif
