/**
 * \file
 * \brief Defines some classes related to a robot state.
 * There are classes RobotState and World.
 */

#ifndef __STATE_DATA_H
#define __STATE_DATA_H

/**
 * \brief Defines a robot position in cartesian coordinates and an angle of
 * rotation.
 */
class RobotState {
public: // methods

  /// Sets a robot in (0,0) oriented as zero angle.
  RobotState(): x(0), y(0), theta(0) {}

  /**
   * Initializes a state of a robot with given parameters.
   * \param x,y,theta The position and the orientation of a robot.
   */
  RobotState(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  /**
   * Updates the state of a robot by the given deltas.
   * \param d_x, d_y, d_theta Delta of the position of a robot.
   */
  void update(double d_x, double d_y, double d_theta) {
    // TODO: move update policy to Strategy.
    // TODO: original gMapping adds a nose on udpate (motionmodel.cpp:13)
    x += d_x;
    y += d_y;
    theta += d_theta;
  }
public:
  double x, y, theta; ///< The position of robot.
};


#include "maps/grid_map.h"

// TODO: try to simplify template params
/**
 * The controller of robot's merged perceptions of an environment.
 */
template <typename ObservationType, typename MapType>
class World {
public:
  // data-in

  /**
   * Sets a new location of a robot.
   * \param x,y,theta New coordinates of a robot.
   */
  virtual void update_robot_pose(double x, double y, double theta) {
    _pose.update(x, y, theta);
  }

  /// Updates a map according to ObservationType data.
  virtual void handle_observation(ObservationType&) = 0;

  // data-out
  /// Returns this world.
  virtual const World<ObservationType, MapType>& world() const { return *this; }

  /// Returns the robot pose.
  virtual const RobotState& pose() const { return _pose; }

  /// Returns the map.
  virtual const MapType& map() const = 0;
private:
  RobotState _pose;
};

#endif
