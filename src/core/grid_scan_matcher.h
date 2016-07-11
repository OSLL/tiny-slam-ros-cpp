#ifndef __GRID_SCAN_MATCHER_H
#define __GRID_SCAN_MATCHER_H

#include <vector>
#include <memory>
#include <algorithm>

#include "state_data.h"
#include "sensor_data.h"

class GridScanMatcherObserver {
public:
  virtual void on_matching_start(const RobotState &,           /*pose*/
                                 const TransformedLaserScan &, /*scan*/
                                 const GridMap &) {}    /*map*/
  virtual void on_scan_test(const RobotState &,           /*pose*/
                            const TransformedLaserScan &, /*scan*/
                            double) {};                  /*score*/
  virtual void on_pose_update(const RobotState &,            /*pose*/
                              const TransformedLaserScan &,  /*scan*/
                              double) {};                    /*score*/
  virtual void on_matching_end(const RobotState &, /*delta*/
                               double) {};         /*best_score*/
};

class ScanCostEstimator {
public:
  virtual double estimate_scan_cost(const RobotState &pose,
                                    const TransformedLaserScan &scan,
                                    const GridMap &map,
                                    double min_cost) = 0;
};

class GridScanMatcher {
public:
  GridScanMatcher(std::shared_ptr<ScanCostEstimator> estimator) :
    _cost_estimator(estimator) {}

  virtual double process_scan(const RobotState &init_pose,
                              const TransformedLaserScan &scan,
                              const GridMap &map,
                              RobotState &pose_delta) = 0;

  virtual void reset_state() {};

  void subscribe(std::shared_ptr<GridScanMatcherObserver> obs) {
    _observers.push_back(obs);
  }

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
  std::shared_ptr<ScanCostEstimator> cost_estimator() {
    return _cost_estimator;
  }

  std::vector<std::weak_ptr<GridScanMatcherObserver>> & observers() {
    return _observers;
  }
private:
  std::vector<std::weak_ptr<GridScanMatcherObserver>> _observers;
  std::shared_ptr<ScanCostEstimator> _cost_estimator;
};

#endif
