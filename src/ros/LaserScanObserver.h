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


#ifndef __LASER_SCAN_OBSERVER_H
#define __LASER_SCAN_OBSERVER_H

#include <utility>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include "../sensor_data.h"
#include "TopicWithTransform.h"

class LaserScanObserver : public TopicObserver<sensor_msgs::LaserScan> {
  using ScanPtr = boost::shared_ptr<sensor_msgs::LaserScan>;
public: //methods

  LaserScanObserver():_prev_x(0), _prev_y(0), _prev_yaw(0) {}

  virtual void handle_transformed_msg(
    const ScanPtr msg, const tf::StampedTransform& t) {

    double new_x = t.getOrigin().getX();
    double new_y = t.getOrigin().getY();
    double new_yaw = tf::getYaw(t.getRotation());

    TransformedLaserScan laser_scan;
    laser_scan.quality = 1.0;
    double angle = msg->angle_min;

    for (const auto &range : msg->ranges) {
      ScanPoint sp(range, angle);
      angle += msg->angle_increment;

      if (sp.range < msg->range_min) {
        continue;
      } else if (msg->range_max <= sp.range) {
        sp.is_occupied = false;
      }
      laser_scan.points.push_back(sp);
    }

    laser_scan.d_x = new_x - _prev_x;
    laser_scan.d_y = new_y - _prev_y;
    laser_scan.d_yaw = new_yaw - _prev_yaw;
    _prev_x = new_x, _prev_y = new_y, _prev_yaw = new_yaw;

    handle_laser_scan(laser_scan);
  }

  virtual void handle_laser_scan(TransformedLaserScan &) = 0;
private: // fields
  double _prev_x, _prev_y, _prev_yaw;
};

#endif
