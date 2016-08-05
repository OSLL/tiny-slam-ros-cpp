#ifndef __LASER_SCAN_OBSERVER_H
#define __LASER_SCAN_OBSERVER_H

#include <utility>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include "../core/sensor_data.h"
#include "topic_with_transform.h"

/**
 * \brief Class responsibilities: observes laser scans and odometry;
 * converts ROS structures to internal representation.
 */
class LaserScanObserver : public TopicObserver<sensor_msgs::LaserScan> {
  using ScanPtr = boost::shared_ptr<sensor_msgs::LaserScan>;
public: //methods
/**
 * Initializes the base laser scan observer.
 * \param skip_max_vals Whether scan points that exceed the max reliable scan-specific distance be skipped.
 */
  LaserScanObserver(bool skip_max_vals = false):
    _skip_max_vals(skip_max_vals),
    _prev_x(0), _prev_y(0), _prev_yaw(0) {}
/**
 * \brief Converts ROS-specific structures that hold sensor data to internal framework's structures;
 * Laser scan filtering is performed as part of the conversion.
 * \param msg A ROS specific laser scan message.
 * \param t A TF specific transform.
 */
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
        sp.range = msg->range_max;
        if (_skip_max_vals) {
          continue;
        }
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
  bool _skip_max_vals;
  double _prev_x, _prev_y, _prev_yaw;
};

#endif
