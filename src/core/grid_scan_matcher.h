/**
 * \file
 * \brief Defines some interfaces for Scan Matchers.
 * There are class GridScanMatcherObserver, ScanCostEstimator, GridScanMatcher.
 */

#ifndef __GRID_SCAN_MATCHER_H
#define __GRID_SCAN_MATCHER_H

#include <vector>
#include <memory>
#include <algorithm>

#include "state_data.h"
#include "sensor_data.h"

/**
 * \brief Interface of scan matcher observer.
 */
class GridScanMatcherObserver {
public:
  /**
   * A callback invoked on scan matching start.
   * \param RobotState A pose of a robot.
   * \param TransformedLaserScan A laser scan with a transformation.
   * \param GridMap A grid map that is used by the matcher.
   */
  virtual void on_matching_start(const RobotState &,           /*pose*/
                                 const TransformedLaserScan &, /*scan*/
                                 const GridMap &) {}    /*map*/

  /**
   * A callback invoked when a new pose is about to be tested.
   * \param RobotState A pose of a robot.
   * \param TransformedLaserScan A laser scan with a transformation.
   */
  virtual void on_scan_test(const RobotState &,           /*pose*/
                            const TransformedLaserScan &, /*scan*/
                            double) {};                  /*score*/

  /**
   * A callback invoked when a given pose better fits to a given scan than
   * the previously estimated one.
   * \param RobotState A pose of a robot.
   * \param TransformedLaserScan A laser scan with a transformation.
   */
  virtual void on_pose_update(const RobotState &,            /*pose*/
                              const TransformedLaserScan &,  /*scan*/
                              double) {};                    /*score*/

  /**
   * A callback invoked when scan matching is done.
   * \param RobotState A pose of a robot.
   */
  virtual void on_matching_end(const RobotState &, /*delta*/
                               double) {};         /*best_score*/
};

/**
 * \brief Interface of Estimator of Scan Cost.
 * Cost - is a number that complies to a scan; the lower cost the better scan.
 */
class ScanCostEstimator {
public:

  /**
   * A callback invoked on estimating the cost of a scan.
   * \param pose A pose of a robot.
   * \param scan A laser scan with a transformation.
   * \param map A grid map that is used by the matcher.
   */
  virtual double estimate_scan_cost(const RobotState &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost) = 0;
};

/**
 * \brief Class that matches scans.
 * Performes a scan adjustment by altering a robot pose in order to maximize
 * the correspondence between a scan and a grid map; the rule of correspondence
 * computation is defined in ScanCostEstimator subclasses.
 */
class GridScanMatcher {
public:
  GridScanMatcher(std::shared_ptr<ScanCostEstimator> estimator) :
    _cost_estimator(estimator) {}

  /**
   * A callback invoked on scan processing.
   * \param init_pose A pose of a robot.
   * \param scan A laser scan with a transformation.
   * \param map A grid map that is used by the matcher.
   * \param pose_delta The difference between the real robot pose and its
   * estimation.
   */
  virtual double process_scan(const RobotState &init_pose,
                              const TransformedLaserScan &scan,
                              const GridMap &map,
                              RobotState &pose_delta) = 0;

  /// Invoked to reset the scan matcher's state.
  virtual void reset_state() {};

  /**
   * Adds an observer.
   * \param obs A shared pointer to an observer to be added.
   */
  void subscribe(std::shared_ptr<GridScanMatcherObserver> obs) {
    _observers.push_back(obs);
  }

  /**
   * Removes an observer.
   * \param obs A shared pointer to an observer to be removed
   */
  void unsubscribe(std::shared_ptr<GridScanMatcherObserver> obs) {
    // TODO: replace with more ideomatic C++
    std::vector<std::weak_ptr<GridScanMatcherObserver>> new_observers;
    for (auto &raw_obs : GridScanMatcher::observers()) {
      auto obs_ptr = raw_obs.lock();
      if (obs_ptr && obs_ptr != obs) {
        new_observers.push_back(raw_obs);
      }
    }
    _observers = new_observers;
  }

protected:
  /// Returns a pointer to the cost estimator.
  std::shared_ptr<ScanCostEstimator> cost_estimator() {
    return _cost_estimator;
  }

  /// Returns a reference to the vector of pointers on observers.
  std::vector<std::weak_ptr<GridScanMatcherObserver>> & observers() {
    return _observers;
  }
private:
  std::vector<std::weak_ptr<GridScanMatcherObserver>> _observers;
  std::shared_ptr<ScanCostEstimator> _cost_estimator;
};

#endif
