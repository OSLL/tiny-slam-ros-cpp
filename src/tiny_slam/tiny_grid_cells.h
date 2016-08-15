/*!
 * \file
 * \brief Definition of classes file:\n
 *   BaseTinyCell, AvgTinyCell (are inherited from GidCell),\n
 *   TinyBaseCellFactory, TinyAvgCellFactory (from GridCellFactory)
 *
 * The classes inherited from GridCell implements different approaches of a
 * grid cell's occupancy maintaining.\n
 * The another ones inherited from GridCellFactory presents factories to
 * produce the cells meant before.
 */
#ifndef __TINY_GRID_CELLS_H
#define __TINY_GRID_CELLS_H

#include "../core/maps/grid_map.h"

//------------------------------------------------------------------------------
// Base cell

/*!
 * \brief The grid cell's model presented in the original tinySLAM paper.
 *
 * The probability is updated by the following rule:
 * \f[p_i=(1-\alpha)\cdot p_{i-1}+\alpha\cdot p_{new}.\f]
 */
class BaseTinyCell : public GridCell {
public:
  /*!
   * Sets the value of the probability \f$p_0=0.5\f$.
   */
  BaseTinyCell(): _prob(0.5) {}
  /*!
   * Returns the estimated probability.
   */
  double value() const override { return _prob; }
  /*!
   * Merges a given probability with the stored one using a quality as a weight.
   * \param[in] value   - a value of the probability \f$p_{new}\f$.
   * \param[in] quality - the quality of the experiment value \f$\alpha\f$.
   */
  void set_value(const Occupancy &value, double quality) override {
    _prob = (1.0 - quality) * _prob + quality * value.prob_occ;
  }
private:
  double _prob;
};

/*!
 * \brief A strategy creates cells of the base tiny model (BaseTinyCell).
 *
 * This class is inherited from an abstract cell factory
 * and generates cells with the base rule of the probability calculation.
 */
class TinyBaseCellFactory : public GridCellFactory {
public:
  /*!
   * Creates a pointer to an instance of BaseTinyCell.
   */
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new BaseTinyCell());
  }
};

//------------------------------------------------------------------------------
// Modified cell

/*!
 * \brief The grid cell's model updates probabilities as an average value.
 *
 * The probability is updated by the following rule:
 * \f[p_n=\frac{p_{n-1}+0.5+(p_{new}-0.5)\cdot\alpha}{n}.\f]
 */
class AvgTinyCell : public GridCell {
public:
  /*!
   * Sets the values of the probability \f$p_0=0\f$ and \f$n=0\f$.
   */
  AvgTinyCell(): _cnt(0), _n(0) {}
  /*!
   * Returns the estimated probability.
   */
  double value() const override { return _n == 0 ? -1 : _cnt / _n; }
  /*!
   * Merges a given probability with the stored one as an average value
   * of all given probabilities also using a quality as a weight.
   * \param[in] value   - a value of the probability \f$p_{new}\f$.
   * \param[in] quality - the quality of the experiment value \f$\alpha\f$.
   */
  void set_value (const Occupancy &value, double quality) override {
    _n += 1;
    _cnt += 0.5 + (value.prob_occ - 0.5) * quality;
  }
private:
  double _cnt, _n;
};

/*!
 * \brief A strategy creates cells with the average probability
 *        calculation rule (AvgTinyCell).
 *
 * This class is inherited from an abstract cell factory and generates cells
 * with the AvgTinyCell model of the probability calculation.
 */
class TinyAvgCellFactory : public GridCellFactory {
public:
  /*!
   * Creates a pointer to an instance of AvgTinyCell.
   */
  std::shared_ptr<GridCell> create_cell() override {
    return std::shared_ptr<GridCell>(new AvgTinyCell());
  }
};

#endif
