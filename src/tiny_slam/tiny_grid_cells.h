/*!
 * \file
 * \brief Definition of classes file: BaseTinyCell, AvgTinyCell (are inherited from GidCell), TinyBaseCellFactory, TinyAvgCellFactory (from GridCellFactory)
 *
 * The classes inherited from GridCell implements different approaches of grid cell's occupancy maintaining.
 * Another ones inherited from GridCellFactory presents factories to produce cells meant before/
 */
#ifndef __TINY_GRID_CELLS_H
#define __TINY_GRID_CELLS_H

#include "../core/maps/grid_map.h"

//------------------------------------------------------------------------------
// Base cell

/*!
 * \brief The grid cell's model presented in original tinySLAM paper.
 *
 * The probability is updated by the following rule:
 * \f[p_i=(1-\alpha)\cdot p_{i-1}+\alpha\cdot p_{new}\f]
 */
class BaseTinyCell : public GridCell {
public:
  /*!
   * Sets the value of probability \f$p_0=0.5\f$
   */
  BaseTinyCell(): _prob(0.5) {}
  /*!
   * Returns estimated probability
   */
  double value() const override { return _prob; }
  /*!
   * Merges a given probability with already stored one using quality as a weight
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
 * \brief A strategy that creates cells of base tiny model (BaseTinyCell)
 *
 * This class is inherited from abstract cell factory and generated cells with base rule of calculation probability.
 */
class TinyBaseCellFactory : public GridCellFactory {
public:
  /*!
   * Creates an instance of BaseTinyCell
   */
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new BaseTinyCell());
  }
};

//------------------------------------------------------------------------------
// Modified cell

/*!
 * \brief The grid cell's model presents calculation probabilities as average value.
 *
 * The probability is updated by the following rule:
 * \f[p_n=\frac{p_{n-1}+0.5+(p_{new}-0.5)\cdot\alpha}{n}\f]
 */
class AvgTinyCell : public GridCell {
public:
  /*!
   * Sets the value of probability \f$p_0=0\f$ and \f$n=0\f$
   */
  AvgTinyCell(): _cnt(0), _n(0) {}
  /*!
   * Returns estimated probability
   */
  double value() const override { return _n == 0 ? -1 : _cnt / _n; }
  /*!
   * Merges a given probability with already stored one as average value of all given probabilities also using quality as a weight
   * \param[in] value   - value of probability \f$p_{new}\f$
   * \param[in] quality - the quality of experiment value \f$\alpha\f$
   * \return the new value of probability of current cell based on the previous probability value of this cell and new value took from laser scanner
   */
  void set_value (const Occupancy &value, double quality) override {
    _n += 1;
    _cnt += 0.5 + (value.prob_occ - 0.5) * quality;
  }
private:
  double _cnt, _n;
};

/*!
 * \brief A strategy that creates cells of base tiny model (AvgTinyCell)
 *
 * This class is inherited from abstract cell factory and generated cells with average rule of calculation probability.
 */
class TinyAvgCellFactory : public GridCellFactory {
public:
  /*!
   * Creates an instance of AvgTinyCell
   */
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new AvgTinyCell());
  }
};

#endif
