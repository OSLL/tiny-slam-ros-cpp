#ifndef __STATE_DATA_H
#define __STATE_DATA_H

class RobotState {
public: // methods
  RobotState(): x(0), y(0), theta(0) {}
  RobotState(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  void update(double d_x, double d_y, double d_theta) {
    // TODO: move update policy to Strategy.
    // TODO: original gMapping adds a nose on udpate (motionmodel.cpp:13)
    x += d_x;
    y += d_y;
    theta += d_theta;
  }
public:
  double x, y, theta;
};


#include "maps/grid_map.h"

// TODO: try to simplify template params
template <typename ObservationType, typename MapType>
class World {
public:
  // data-in
  virtual void update_robot_pose(double x, double y, double theta) {
    _pose.update(x, y, theta);
  }

  virtual void handle_observation(ObservationType&) = 0;

  // data-out
  virtual const World<ObservationType, MapType>& world() const { return *this; }
  virtual const RobotState& pose() const { return _pose; }
  virtual const MapType& map() const = 0;
private:
  RobotState _pose;
};

#endif
