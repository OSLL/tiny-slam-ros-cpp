/**
 * \file
 * \brief Discribes the class of Scan matcher using Mote-Carlo method
 * There is class MonteCarloScanMatcher  derived from GridScanMatcher. Class contains pure virtual methods
 * The main method consists of loop that should realize the best position of robot that suits to data from laser scan
 */


#ifndef __MONTE_CARLO_SCAN_MATCHER_H
#define __MONTE_CARLO_SCAN_MATCHER_H

#include <functional>
#include <limits>
#include <memory>

#include "state_data.h"
#include "sensor_data.h"
#include "grid_scan_matcher.h"

// TODO: move publish transform to observer

/**
 * \brief Scan Matcher that uses Mone Carlo Method
 * The focus of scan matcher is to compare a scan and a build map. In this class the method of comparation is based on Mone-Carlo method
 * 
 */
class MonteCarloScanMatcher : public GridScanMatcher {
public:
  using ObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public:
  /**
   * \brief Constructor with parameters.
   * \param estimator Current estimator of Scan Cost
   * \param failed_iter Shows max amount of failed tries allowed
   * \param max_iter Shows max amount of total tries allowed
   */
  MonteCarloScanMatcher(std::shared_ptr<ScanCostEstimator> estimator,
                        unsigned failed_iter, unsigned max_iter):
    GridScanMatcher{estimator},
    _failed_tries_limit(failed_iter), _total_tries_limit(max_iter) {}

  /**
   * initiates estimation of scan after changing sample_pose.
   * First of all the sample pose of robot is set. Than the cost of current scan is estimated. than the sample pose of robot is changed and operation repears.
   * Repeating stops when the limit of failed tries or total tries is reached. The pose where scan cost is lowest, is optimal.
   * \param init_pose The first approxiamtion of pose
   * \param scan Current scan
   * \param map Current GridMap
   * \param pose_delta Output parametr of the best pose_delta
   * \return The lowest scan cost that complies to output pose_delta
   */
  virtual double process_scan(const RobotState &init_pose,
                      const TransformedLaserScan &scan,
                      const GridMap &map,
                      RobotState &pose_delta) override {
    do_for_each_observer([init_pose, scan, map](ObsPtr obs) {
      obs->on_matching_start(init_pose, scan, map);
    });

    unsigned failed_tries = 0, total_tries = 0;
    std::shared_ptr<ScanCostEstimator> sce = GridScanMatcher::cost_estimator();
    double min_scan_cost = std::numeric_limits<double>::max();
    RobotState optimal_pose = init_pose;

    min_scan_cost = sce->estimate_scan_cost(optimal_pose, scan,
                                            map, min_scan_cost);

    do_for_each_observer([optimal_pose, scan, min_scan_cost](ObsPtr obs) {
      obs->on_scan_test(optimal_pose, scan, min_scan_cost);
      obs->on_pose_update(optimal_pose, scan, min_scan_cost);
    });

    while (failed_tries < _failed_tries_limit &&
           total_tries < _total_tries_limit) {
      total_tries++;

      RobotState sampled_pose = optimal_pose;
      sample_pose(sampled_pose);
      double sampled_scan_cost = sce->estimate_scan_cost(sampled_pose, scan,
                                                         map, min_scan_cost);
      do_for_each_observer([sampled_pose, scan, sampled_scan_cost](ObsPtr obs) {
        obs->on_scan_test(sampled_pose, scan, sampled_scan_cost);
      });

      if (min_scan_cost <= sampled_scan_cost) {
        failed_tries++;
        continue;
      }

      min_scan_cost = sampled_scan_cost;
      optimal_pose = sampled_pose;
      failed_tries = on_estimate_update(failed_tries, _failed_tries_limit);

      do_for_each_observer([optimal_pose, scan, min_scan_cost](ObsPtr obs) {
        obs->on_pose_update(optimal_pose, scan, min_scan_cost);
      });
    }

    pose_delta.x = optimal_pose.x - init_pose.x;
    pose_delta.y = optimal_pose.y - init_pose.y;
    pose_delta.theta = optimal_pose.theta - init_pose.theta;

    do_for_each_observer([pose_delta, min_scan_cost](ObsPtr obs) {
        obs->on_matching_end(pose_delta, min_scan_cost);
    });
    return min_scan_cost;
  }

protected:
  /**
   * Virtual function. Will change pose of robot relatively base_pose
   * \param base_pose Basical pose of robot
   */
  virtual void sample_pose(RobotState &base_pose) = 0;
  /**
   * Virtual fuction. Will be called when the better estimate is found
   * \param sample_num Amount of tries that were complited to the current moment
   * \param sample_limit Totla amount of tries allowed
   */
  virtual unsigned on_estimate_update(unsigned sample_num,
                                      unsigned sample_limit) = 0;

private:
  /**
   * Function that allowes to do any function to each observer.
   * \param op Function that has Observer as input parameter
   */
  void do_for_each_observer(std::function<void(ObsPtr)> op) {
    for (auto &obs : GridScanMatcher::observers()) {
      if (auto obs_ptr = obs.lock()) {
        op(obs_ptr);
      }
    }
  }

private:
  unsigned _failed_tries_limit;
  unsigned _total_tries_limit;
};

#endif
