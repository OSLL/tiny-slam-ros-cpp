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

#ifndef __STATE_DATA_H
#define __STATE_DATA_H

class RobotState {
public: // methods
  RobotState(): x(0), y(0), theta(0) {}
  RobotState(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  void update(double d_x, double d_y, double d_theta) {
    // TODO: move update policy to Strategy.
    x += d_x;
    y += d_y;
    theta += d_theta;
  }
public:
  double x, y, theta;
};

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
  virtual const RobotState& pose() const { return _pose; }
  virtual const MapType& map() const = 0;
private:
  RobotState _pose;
};

#endif
