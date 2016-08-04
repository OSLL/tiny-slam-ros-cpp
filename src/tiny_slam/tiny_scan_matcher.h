#ifndef __TINY_SCAN_MATCHER_H
#define __TINY_SCAN_MATCHER_H

#include <random>

#include "../core/monte_carlo_scan_matcher.h"
#include "tiny_scan_cost_estimator.h"

/*!
 * \brief The scan matcher based on the Monte Carlo simulation.
 *
 * The robot pose is updated by shifting it on a random vector and
 * rotation by a random angle. The vector distribution is dynamically adjusted.
 */
class TinyScanMatcher : public MonteCarloScanMatcher {
private:
  using ScePtr = std::shared_ptr<ScanCostEstimator>;
public:
  /*!
   * Initializes the scan matcher.
   * \param[in] cost_estimator - the type of estimator for the robot location.
   * \param[in] bad_iter, max_iter - see <b>failed_iter</b>, <b>max_iter</b> in
   *                                 MonteCarloScanMatcher
   * \param[in] sigma_coord, sigma_angle - the \f$\sigma\f$ value of a normal
   *                                      distribution for the random variables.
   *                   (\f$\Delta x\f$, \f$\Delta y\f$ and \f$\Delta \theta\f$).
   */
  TinyScanMatcher(ScePtr cost_estimator, unsigned bad_iter, unsigned max_iter,
                  double sigma_coord, double sigma_angle):
   MonteCarloScanMatcher(cost_estimator, bad_iter, max_iter),
    _sigma_coord(sigma_coord), _sigma_angle(sigma_angle),
    _curr_sigma_coord(_sigma_coord), _curr_sigma_angle(_sigma_angle) {}

  /*!
   * Resets the scan matcher to the state it had right after the initialization.
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
