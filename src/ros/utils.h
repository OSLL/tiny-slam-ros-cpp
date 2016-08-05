#ifndef __SLAM_FMWK_ROS_UTILS_H
#define __SLAM_FMWK_ROS_UTILS_H

#include <string>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>


inline void publish_2D_transform(const std::string &target_frame,
                                 const std::string &base_frame,
                                 double x, double y, double th) {

  tf::Vector3 translation(x, y, 0);
  tf::Quaternion rotation;
  rotation.setRPY(0, 0, th);

  tf::Transform tr(rotation, translation);
  tf::StampedTransform st_trans(tr, ros::Time::now(), base_frame, target_frame);

  static tf::TransformBroadcaster br;
  br.sendTransform(st_trans);
}

#endif
