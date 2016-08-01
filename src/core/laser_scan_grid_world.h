/**
 * \file
 * \brief Defines the special type of World
 * There is class LaserScanGridWorld derived from World
 */

#ifndef __LASER_SCAN_GRID_WORLD_H
#define __LASER_SCAN_GRID_WORLD_H

#include <memory>

#include "sensor_data.h"
#include "state_data.h"
#include "maps/grid_cell_strategy.h"
#include "maps/grid_map.h"

/**
 * \brief Tracks a robots perception of an environment
 * The environment is represented by a GridMap; laser scan with transformation
 * is expected as sensor data
 */
class LaserScanGridWorld : public World<TransformedLaserScan, GridMap> {
public: //types
  using MapType = GridMap;
  using Point = DiscretePoint2D;
public: // methods

  /**
   * Creates a world as a Map of Grid Cells
   * \param gcs Shared pointer on Cell
   */
  LaserScanGridWorld(std::shared_ptr<GridCellStrategy> gcs) :
    _map(gcs->cell_factory()) {}

  /**
   * Updates the map cells according to given sensor data. Straightforward scan
   * points projection is used
   * \param scan Current scan from Laser Rangefinder
   */
  virtual void handle_observation(TransformedLaserScan &scan) {
    const RobotState& pose = World<TransformedLaserScan, MapType>::pose();
    for (const auto &sp : scan.points) {
      // move to world frame assume sensor is in robots' (0,0)
      double x_world = pose.x + sp.range * std::cos(sp.angle + pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle + pose.theta);

      handle_scan_point(map(), pose.x, pose.y, x_world, y_world,
                        sp.is_occupied, scan.quality);
    }
  }

  /**
   * Udates cells of map according to given parameters
   * \param map Current map
   * \param laser_x,lasery Coordinate of Laser
   * \param beam_end_x,beam_end_y Coordinates of a current point on scan
   * \is_occ Current assumption whether the cell is occupied
   * \quality Quality of knowledge
   */
  virtual void handle_scan_point(MapType &map,
                                 double laser_x, double laser_y,
                                 double beam_end_x, double beam_end_y,
                                 bool is_occ, double quality) {
    Point robot_pt = map.world_to_cell(laser_x, laser_y);
    Point obst_pt = map.world_to_cell(beam_end_x, beam_end_y);

    std::vector<Point> pts = DiscreteLine2D(robot_pt, obst_pt).points();

    map.update_cell(pts.back(), Occupancy{is_occ ? 1.0 : 0.0, 1.0}, quality);
    pts.pop_back();
    for (const auto &pt : pts) {
      map.update_cell(pt, Occupancy{0.0, 1.0}, quality);
    }
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

private: // fields
  MapType _map;
};

#endif
