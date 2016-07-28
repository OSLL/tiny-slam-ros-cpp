#ifndef __TINY_SCAN_MATCHER_H
#define __TINY_SCAN_MATCHER_H

#include <random>

#include "../core/monte_carlo_scan_matcher.h"
#include "tiny_scan_cost_estimator.h"

/*!
 * \brief Derived class from MonteCarloScanMatcher to replace robot pose in space
 *
 * There is a logic how to choose a place of robot in the world (by generating vector of randomize variables - shifts of robot coordinates in the space);
 * and how to change the search area when good place has been found.
 */
class TinyScanMatcher : public MonteCarloScanMatcher {
private:
  using ScePtr = std::shared_ptr<ScanCostEstimator>;
public:
  /*!
   * Parameterized constructor sets all data members
   * \param[in] cost_estimator           - the type of estimator for robot location
   * \param[in] bad_iter                 - max amount of failed iteration while it is not found better place for robot
   * \param[in] max_iter                 - max amount of all iteration while it is tried to find fine place for robot
   * \param[in] sigma_coord, sigma_angle - the \f$\sigma\f$ value of normal distribution for random variables (\f$\Delta x\f$, \f$\Delta y\f$ and \f$\Delta \theta\f$)
   */
  TinyScanMatcher(ScePtr cost_estimator, unsigned bad_iter, unsigned max_iter,
                  double sigma_coord, double sigma_angle):
   MonteCarloScanMatcher(cost_estimator, bad_iter, max_iter),
    _sigma_coord(sigma_coord), _sigma_angle(sigma_angle),
    _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle) {}

  /*!
   * Function resets current value of \f$\sigma_{x,y,\theta}\f$ to its initial value
   */
  virtual void reset_state() override {
    _curr_sigma_coord = _sigma_coord;
    _curr_sigma_angle = _sigma_angle;
  }

protected:
  virtual void sample_pose(RobotState &base_pose) override {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_coord(0.0, _curr_sigma_coord);
    std::normal_distribution<> d_angle(0.0, _curr_sigma_angle);

    base_pose.x += d_coord(gen);
    base_pose.y += d_coord(gen);
    base_pose.theta += d_angle(gen);
  }

  virtual unsigned on_estimate_update(unsigned sample_num,
                                      unsigned sample_limit) override {
    if (sample_num <= sample_limit / 3) {
      return sample_num;
    }

    _curr_sigma_coord *= 0.5;
    _curr_sigma_angle *= 0.5;
    return 0;
  }

private:
  double _sigma_coord, _sigma_angle;
  double _curr_sigma_coord, _curr_sigma_angle;
};

#endif
