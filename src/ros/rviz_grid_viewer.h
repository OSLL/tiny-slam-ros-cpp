/**
 * \file
 * \brief In this file, is implemented a class that can show the map and the position of the robot.
 */

#ifndef __RVIZ_GRID_VIEWER_H
#define __RVIZ_GRID_VIEWER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../core/state_data.h"
#include "../core/maps/grid_map.h"

/**
 * \brief This class show the map and robot position on this map.
 */

class RvizGridViewer {
public: // method
  RvizGridViewer(ros::Publisher pub):
    _map_pub(pub) {}

  /**
   * This function shows robot's position.
   * \param r stores robot's position.
   */

  void show_robot_pose(const RobotState &r) {
    tf::Transform t;
    t.setOrigin(tf::Vector3(r.x, r.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, r.theta);
    t.setRotation(q);
    _tf_brcst.sendTransform(
      tf::StampedTransform(t, ros::Time::now(),
                           "odom_combined", "robot_pose"));
  }

  /**
   * This function shows the map.
   * \param map stores the map.
   */

  void show_map(const GridMap &map) {
    // TODO: move map publishing rate to parameter
    if ((ros::Time::now() - _last_pub_time).toSec() < 5.0) {
      return;
    }

    nav_msgs::OccupancyGrid map_msg;
    map_msg.info.map_load_time = ros::Time::now();
    map_msg.info.width = map.width();
    map_msg.info.height = map.height();
    map_msg.info.resolution = map.scale();
    // move map to the middle
    nav_msgs::MapMetaData &info = map_msg.info;
    info.origin.position.x = -info.resolution * info.height / 2;
    info.origin.position.y = -info.resolution * info.width  / 2;
    info.origin.position.z = 0;

    for (const auto &row : map.cells()) {
      for (const auto &cell : row) {
        int cell_value = cell->value() == -1 ? -1 : cell->value() * 100;
        map_msg.data.push_back(cell_value);
      }
    }

    _map_pub.publish(map_msg);
    _last_pub_time = ros::Time::now();
  }

private: // fields
  ros::Publisher _map_pub; ///< This data member stores map publisher by ROS
  ros::Time _last_pub_time;  ///< This data member stores last map publisher time.
  tf::TransformBroadcaster _tf_brcst; ///< This data member store publish information of coordinate frame transform
};

#endif
