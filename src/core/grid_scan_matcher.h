/**
 * \file
 * \brief Discribes some interfaces for Scan Matchers
 * There are class GridScanMatcherObserver, ScanCostEstimator, GridScanMatcher
 */

#ifndef __GRID_SCAN_MATCHER_H
#define __GRID_SCAN_MATCHER_H

#include <vector>
#include <memory>
#include <algorithm>

#include "state_data.h"
#include "sensor_data.h"

/**
 * \brief Interface of observer on Scan Matcher
 */
class GridScanMatcherObserver {
public:
  /**
   * Virtual function. Starts matching.
   * \param RobotState Pose of a robot
   * \param TransformedLaserScan Information about a scan
   * \param GridMap Information about a map
   */
  virtual void on_matching_start(const RobotState &,           /*pose*/
                                 const TransformedLaserScan &, /*scan*/
                                 const GridMap &) {}    /*map*/
  /**
   * Virtual function. Tests the scan
   * \param RobotState Pose of a robot
   * \param TransformedLaserScan Information about a scan
   * \param double Score
   */
  virtual void on_scan_test(const RobotState &,           /*pose*/
                            const TransformedLaserScan &, /*scan*/
                            double) {};                  /*score*/
  /**
   * Virtual function. Updates a pose of a robot
   * \param RobotState Pose of a robot
   * \param TransformedLaserScan Information about a scan
   * \param double Score
   */
  virtual void on_pose_update(const RobotState &,            /*pose*/
                              const TransformedLaserScan &,  /*scan*/
                              double) {};                    /*score*/
  /*
   * Virtual function. Stops matching
   * \param RobotState Pose of a robot
   * \param double Score
   */
	 virtual void on_matching_end(const RobotState &, /*delta*/
                               double) {};         /*best_score*/
};

/**
 * \brief Interface of Estimator of Scan Cost.
 * Cost - is a number that complies to a scan. As greater cost than worser scan
 */
class ScanCostEstimator {
public:

  /**
   * Estimates the cost of a scan
   * \param pose Pose of a robot
   * \param scan Information about a scan
   * \param map - Information about a map
   */
  virtual double estimate_scan_cost(const RobotState &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost) = 0;
};

/**
 * \brief  Class that matches scans
 * When a new scan appears, it is requied to compare it with a current status of World. This class is to handle these situations
 */
class GridScanMatcher {
public:
  GridScanMatcher(std::shared_ptr<ScanCostEstimator> estimator) :
    _cost_estimator(estimator) {}
/**
 * virtual function that matches every current scan
 * \param init_pose Pose of a robot
 * \param scan Information about scan
 * \param map Information about map
 * \pose_delta Difference between real robot pose and estimation of pose
 */
  virtual double process_scan(const RobotState &init_pose,
                              const TransformedLaserScan &scan,
                              const GridMap &map,
                              RobotState &pose_delta) = 0;
  /**
   * virtual function that resets state of something
   */
  virtual void reset_state() {};

  /**
   * Adds an observer
   */
  void subscribe(std::shared_ptr<GridScanMatcherObserver> obs) {
    _observers.push_back(obs);
  }
  /**
   * Removes an observer
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
  /**
   * Getter of pointer on cost_estimator
   */
  std::shared_ptr<ScanCostEstimator> cost_estimator() {
    return _cost_estimator;
  }

  /**
   * Getter of a link on pointer on vector of observers
   */
  std::vector<std::weak_ptr<GridScanMatcherObserver>> & observers() {
    return _observers;
  }
private:
  std::vector<std::weak_ptr<GridScanMatcherObserver>> _observers;
  std::shared_ptr<ScanCostEstimator> _cost_estimator;
};

#endif
