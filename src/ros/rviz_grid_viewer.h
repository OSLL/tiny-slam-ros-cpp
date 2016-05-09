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


#ifndef __RVIZ_GRID_VIEWER_H
#define __RVIZ_GRID_VIEWER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../state_data.h"
#include "../grid_map.h"

class RvizGridViewer {
public: // method
  RvizGridViewer(ros::Publisher pub):
    _map_pub(pub) {}

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

  template <typename CellType>
  void show_map(const GridMap<CellType> &map) {
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
        int cell_value = cell.value() == -1 ? -1 : cell.value() * 100;
        map_msg.data.push_back(cell_value);
      }
    }

    _map_pub.publish(map_msg);
    _last_pub_time = ros::Time::now();
  }

private: // fields
  ros::Publisher _map_pub;
  ros::Time _last_pub_time;
  tf::TransformBroadcaster _tf_brcst;
};

#endif
