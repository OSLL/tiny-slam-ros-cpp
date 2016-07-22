/*!
 * \file
 *  \brief Description of class file (TinyScanMatcher is inherited from MonteCarloScanMatcher)
 *
 *  This file includes one class TinyScanMatcher and such files like: monte_carlo_scan_matcher.h, tiny_scan_cost_estimator.h
 */
#ifndef __TINY_SCAN_MATCHER_H
#define __TINY_SCAN_MATCHER_H

#include <random>

#include "../core/monte_carlo_scan_matcher.h"
#include "tiny_scan_cost_estimator.h"

/*!
 * \brief Derived class from MonteCarloScanMatcher to replace ropot pose in space
 *
 * This class overrides the function which calculates the new pose of robot to find the cost of new location further
 */
class TinyScanMatcher : public MonteCarloScanMatcher {
private:
  using ScePtr = std::shared_ptr<ScanCostEstimator>;
public:
  /*!
   * Parameterized constructor sets all data members
   * \param[in] cost_estimator, bad_iter, max_iter - data member from base class MonteCarloScanMatcher
   *  \param[in] sigma_coord, sigma_angle - the \f$\sigma\f$ value of normal distribution for random variables (\f$\Delta x\f$, \f$\Delta y\f$ and \f$\Delta \theta\f$)
   */
  TinyScanMatcher(ScePtr cost_estimator, unsigned bad_iter, unsigned max_iter,
                  double sigma_coord, double sigma_angle):
   MonteCarloScanMatcher(cost_estimator, bad_iter, max_iter),
    _sigma_coord(sigma_coord), _sigma_angle(sigma_angle),
    _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle) {}

  /*!
   * Function moved the base parameters for distribution \f$\sigma_{x,y,\theta}\f$ to its current value
   */
  virtual void reset_state() override {
    _curr_sigma_coord = _sigma_coord;
    _curr_sigma_angle = _sigma_angle;
  }

protected:
  /*!
   * Function generates three normal distributed random variables for robot position (\f$\Delta x\f$, \f$\Delta y\f$ and \f$\Delta \theta\f$) and shifts robot pose on these values
   * \param[in] base_pose - initial pose of robot
   */
  virtual void sample_pose(RobotState &base_pose) override {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_coord(0.0, _curr_sigma_coord);
    std::normal_distribution<> d_angle(0.0, _curr_sigma_angle);

    base_pose.x += d_coord(gen);
    base_pose.y += d_coord(gen);
    base_pose.theta += d_angle(gen);
  }

  /*!
   * Function called when good position was found and derives values of \f$sigma\f$ by 2
   * \param[in] sample_num - the amount of algorithm steps before good position was found
   *  \param[in] sample_limit - the total amount of bad position which supported by configuration of cirrent system
   *   \return The refind amount of faild steps (0 or if there were a few amounts of steps, there will be returned this amount)
   */
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
  double _sigma_coord, _sigma_angle; ///< base values of \f$\sigma_{x,y,\theta}\f$
  double _curr_sigma_coord, _curr_sigma_angle; ///< values of \f$\sigma_{x,y,\theta}\f$ on the current step
};

#endif
