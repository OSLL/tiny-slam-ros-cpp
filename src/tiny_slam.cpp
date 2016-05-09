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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

#include <nav_msgs/OccupancyGrid.h>

#include "ros/TopicWithTransform.h"
#include "ros/rviz_grid_viewer.h"
#include "tiny_fascade.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tinySLAM");

  ros::NodeHandle nh;
  std::shared_ptr<TinySlamFascade> slam{new TinySlamFascade()};
  TopicWithTransform<sensor_msgs::LaserScan> scan_observer(nh,
      "laser_scan", "odom_combined");
  scan_observer.subscribe(slam);

  std::shared_ptr<RvizGridViewer> viewer(
    new RvizGridViewer(nh.advertise<nav_msgs::OccupancyGrid>("/map", 5)));
  slam->set_viewer(viewer);

  ros::spin();
}
