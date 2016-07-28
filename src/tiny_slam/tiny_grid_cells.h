/*!
 * \file
 * \brief Definition of classes file: BaseTinyCell, AvgTinyCell (are inherited from GidCell), TinyBaseCellFactory, TinyAvgCellFactory (from GridCellFactory)
 *
 * Two classes from this file (inherited from GridCell) presents two different ways to calculate a probability of cell occupation.
 * Two another ones (inherited from GridCellFactory)presents factories to produce cells meant before/
 */

#ifndef __TINY_GRID_CELLS_H
#define __TINY_GRID_CELLS_H

#include "../core/maps/grid_map.h"

//------------------------------------------------------------------------------
// Base cell
/*!
 * \brief Class for calculated probability with way presented in article
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
   * Getter for value of probability \f$p_i\f$
   * \return current value \f$p_i\f$ of current cell
   */
  double value() const override { return _prob; }
  /*!
   * Modifier which updates value \f$p_i\f$ using previous value \f$p_{i-1}\f$, new probability value \f$p_{new}\f$ and input quality of experiment value\f$\alpha\f$
   * with \f$p_i=(1-\alpha)\cdot p_{i-1}+\alpha\cdot p_{new}\f$
   * \param[in] value   - value of probability \f$p_{new}\f$
   * \param[in] quality - the quality of experiment value \f$\alpha\f$
   * \return the new value of probability of current cell based on the previous probability value of this cell and new value took from laser scanner
   */
  void set_value(const Occupancy &value, double quality) override {
    _prob = (1.0 - quality) * _prob + quality * value.prob_occ;
  }
private:
  double _prob;
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
   * Getter for value of probability \f$p_i\f$
   * \return current value \f$p_i\f$ of current cell
   */
  double value() const override { return _n == 0 ? -1 : _cnt / _n; }
  /*!
   * Function-modifier which updates value \f$p_i\f$ using previous value \f$p_{i-1}\f$, new probability value \f$p_{new}\f$ and input quality of experiment value \f$\alpha\f$
   * with \f$p_n=\frac{p_{n-1}+0.5+(p_{new}-0.5)\cdot\alpha}{n}\f$ (the \f$n\f$ parameter is incremented with every call of this function)
   * \param[in] value   - value of probability \f$p_{new}\f$
   * \param[in] quality - the quality of experiment value \f$\alpha\f$
   * \return the new value of probability of current cell based on the previous probability value of this cell and new value took from laser scanner
   */
  void set_value (const Occupancy &value, double quality) override {
    _n += 1;
    _cnt += 0.5 + (value.prob_occ - 0.5) * quality;
  }
private:
  double _cnt;
  double _n;
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
