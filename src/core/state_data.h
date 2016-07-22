/**
 * \file
 * \brief Discribes some classes related to Robot
 * There are classes RobotState and World
 */

#ifndef __STATE_DATA_H
#define __STATE_DATA_H

/**
 * \brief Class showing robot pose. Cartesian coordinates anf angle of rotation
 */
class RobotState {
public: // methods
  /// Default constructors. Sets roboy in (0,0) oriented as zero angle
  RobotState(): x(0), y(0), theta(0) {}
  /**
   * Costructor with parameters. All parameters are requied
   * \param x,y,theta position and orientation of robot
   */
  RobotState(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  /**
   * increments position of robot on \param d_x, d_y, d_theta
   */
  void update(double d_x, double d_y, double d_theta) {
    // TODO: move update policy to Strategy.
    // TODO: original gMapping adds a nose on udpate (motionmodel.cpp:13)
    x += d_x;
    y += d_y;
    theta += d_theta;
  }
public:
  double x, y, theta; ///< Position of robot
};


#include "maps/grid_map.h"

// TODO: try to simplify template params
/**
 * Class that knows everything about robot state, can update its position and handle observation
 */
template <typename ObservationType, typename MapType>
class World {
public:
  // data-in
  /**
   * Sets new location of robot
   * \param x,y,theta new coordinates of robot
   */
  virtual void update_robot_pose(double x, double y, double theta) {
    _pose.update(x, y, theta);
  }
  /// virtual function that will handle observation depending on Observation type
  virtual void handle_observation(ObservationType&) = 0;

  // data-out
  /// Getter of this world
  virtual const World<ObservationType, MapType>& world() const { return *this; }
  /// Getter of current robot pose
  virtual const RobotState& pose() const { return _pose; }
  /// Getter of map
  virtual const MapType& map() const = 0;
private:
  RobotState _pose; ///< Robot pose
};

#endif
