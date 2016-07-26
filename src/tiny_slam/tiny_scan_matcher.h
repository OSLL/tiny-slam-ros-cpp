/*!
 * \file
 * \brief Description of class file (TinyScanMatcher is inherited from MonteCarloScanMatcher)
 *
 * This file includes one class TinyScanMatcher which solves to generate new pose for robot in local area and to modify a ramdon distribution values.
 */
#ifndef __TINY_SCAN_MATCHER_H
#define __TINY_SCAN_MATCHER_H

#include <random>

#include "../core/monte_carlo_scan_matcher.h"
#include "tiny_scan_cost_estimator.h"

/*!
 * \brief Derived class from MonteCarloScanMatcher to replace robot pose in space
 *
 * This class presents a logic of a monte-carlo step in scan matcher work. Scan matcher tries to find the best location where got data from laser scanner is the most suitable to the environment.
 * So this class presents logic what will happen when this robot location will have found.
 * Besides there overrides the function which calculates the new pose of robot to find the cost of new location further
 */
class TinyScanMatcher : public MonteCarloScanMatcher {
private:
  using ScePtr = std::shared_ptr<ScanCostEstimator>;
public:
  /*!
   * Parameterized constructor sets all data members
   * \param[in] cost_estimator, bad_iter, max_iter - data member from base class MonteCarloScanMatcher
   * \param[in] sigma_coord, sigma_angle           - the \f$\sigma\f$ value of normal distribution for random variables (\f$\Delta x\f$, \f$\Delta y\f$ and \f$\Delta \theta\f$)
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
   * Function generates three normal distributed random variables for robot position (\f$\Delta x\f$, \f$\Delta y\f$ and \f$\Delta \theta\f$)
   * with distribution \f$N(0,\sigma_{x,y,\theta})\f$ and shifts robot pose (\f$x, y, \theta\f$) on these values
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
   * If amount of steps before good position has been found is small than there is no changes in \f$\sigma_{x,y,\theta}\f$ value happen and it is returned the number of these steps.
   * Else the \f$\sigma_{x,y,\theta}\f$ value is derived by 2 and it is flushed the amount of steps to 0.
   *
   * \param[in] sample_num   - the amount of algorithm steps before good position was found
   * \param[in] sample_limit - the total amount of bad position which supported by configuration of current system
   * \return The refind amount of failed steps (0 or if there were a few amounts of steps, there will be returned this amount)
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
