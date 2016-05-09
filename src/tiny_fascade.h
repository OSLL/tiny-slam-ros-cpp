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

#ifndef __TINY_SLAM_FASCADE_H
#define __TINY_SLAM_FASCADE_H

#include "ros/LaserScanObserver.h"
#include "ros/rviz_grid_viewer.h"

#include "tiny_world.h"

class TinySlamFascade : public LaserScanObserver {
public: // methods
  TinySlamFascade():_world(new TinyWorld()) {}

  void set_viewer(std::shared_ptr<RvizGridViewer> viewer) {
    _viewer = viewer;
  }

  virtual void handle_laser_scan(TransformedLaserScan &scan) {
    _world->update_robot_pose(scan.d_x, scan.d_y, scan.d_yaw);
    _world->handle_observation(scan);

    if (_viewer) {
      _viewer->show_robot_pose(_world->pose());
      _viewer->show_map(_world->map());
    }
  }

private:
  std::unique_ptr<World<TransformedLaserScan, TinyWorld::MapType>> _world;
  std::shared_ptr<RvizGridViewer> _viewer;
};


#endif
