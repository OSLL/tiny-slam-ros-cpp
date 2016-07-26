/*!
 * \file
 * \brief Description of four class file: BaseTinyCell, AvgTinyCell (are inherited from GidCell), TinyBaseCellFactory, TinyAvgCellFactory (from GridCellFactory)
 *
 * There are four classes in this file two of them (BaseTinyCell, AvgTinyCell) are inherited from GridCell
 * and presents two different ways to calculate a probability of cell occupation. Two another ones
 * (TinyBaseCellFactory, TinyAvgCellFactory are inherited from GridCellFactory) are to present a factory
 * to produce cells described with previous classes.
 */

#ifndef __TINY_GRID_CELLS_H
#define __TINY_GRID_CELLS_H

#include "../core/maps/grid_map.h"

//------------------------------------------------------------------------------
// Base cell
/*!
 * \brief Class for standard-calculated probability
 *
 * There could be generated grid cells with simple rule of calculating probability:
 * \f[p_i=(1-\alpha)\cdot p_{i-1}+\alpha\cdot p_{new}\f]
 */
class BaseTinyCell : public GridCell {
public:
  /*!
   * Constructor sets the value of probability \f$p_0=0.5\f$
   */
  BaseTinyCell(): _prob(0.5) {}
  /*!
   * Function-getter for value of probability \f$p_i\f$
   * \return current value \f$p_i\f$ of current cell
   */
  double value() const override { return _prob; }
  /*!
   * Function-modifier which recalculate value \f$p_i\f$ using previous value \f$p_{i-1}\f$, new probability value \f$p_{new}\f$ and input weight \f$\alpha\f$
   * with \f$p_i=(1-\alpha)\cdot p_{i-1}+\alpha\cdot p_{new}\f$
   * \param[in] value   - value of probability \f$p_{new}\f$
   * \param[in] quality - the weight value \f$\alpha\f$
   * \return the new value of probability of current cell based on the previous probability value of this cell and new value took from laser scanner
   */
  void set_value(const Occupancy &value, double quality) override {
    _prob = (1.0 - quality) * _prob + quality * value.prob_occ;
  }
private:
  double _prob; ///< the value of probability of current cell
};

/*!
 * \brief Class-factory of BaseTinyCell-s
 *
 * This class is inherited from abstract cell factory and generated cells with base rule of calculation probability:
 * \f[p_i=(1-\alpha)\cdot p_{i-1}+\alpha\cdot p_{new}\f]
 */
class TinyBaseCellFactory : public GridCellFactory {
public:
  /*!
   * Overrided function to create a cell
   * \return shared pointer on new generated cell
   */
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new BaseTinyCell());
  }
};

//------------------------------------------------------------------------------
// Modified cell

/*!
 * \brief Class for calculation probability using average value
 *
 * There could be generated grid cells with a rule of calculating probability:
 * \f[p_n=\frac{p_{n-1}+0.5+(p_{new}-0.5)\cdot\alpha}{n}\f]
 */
class AvgTinyCell : public GridCell {
public:
  /*!
   * Constructor sets the value of probability \f$p_0=0\f$ and \f$n=0\f$
   */
  AvgTinyCell(): _cnt(0), _n(0) {}
  /*!
   * Function-getter for value of probability \f$p_i\f$
   * \return current value \f$p_i\f$ of current cell
   */
  double value() const override { return _n == 0 ? -1 : _cnt / _n; }
  /*!
   * Function-modifier which recalculate value \f$p_i\f$ using previous value \f$p_{i-1}\f$, new probability value \f$p_{new}\f$ and input weight \f$\alpha\f$
   * with \f$p_n=\frac{p_{n-1}+0.5+(p_{new}-0.5)\cdot\alpha}{n}\f$ (the \f$n\f$ parameter increments with every call of this function)
   * \param[in] value   - value of probability \f$p_{new}\f$
   * \param[in] quality - the weight value \f$\alpha\f$
   * \return the new value of probability of current cell based on the previous probability value of this cell and new value took from laser scanner
   */
  void set_value (const Occupancy &value, double quality) override {
    _n += 1;
    _cnt += 0.5 + (value.prob_occ - 0.5) * quality;
  }
private:
  double _cnt; ///< the value of cell probability multiplied on total amount of experiments \f$p_i \cdot n\f$
  double _n; ///< the amount of experiments \f$n\f$ where probability of current cell be occupied is calculated
};
/*!
 * \brief Class-factory of AvgTinyCell-s
 *
 * This class is inherited from abstract cell factory and generated cells with average rule of calculation probability:
 * \f[p_n=\frac{p_{n-1}+0.5+(p_{new}-0.5)\cdot\alpha}{n}\f]
 */
class TinyAvgCellFactory : public GridCellFactory {
public:
  /*!
   * Overrided function to create a cell
   * \return shared pointer on new generated cell
   */
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new AvgTinyCell());
  }
};

#endif
