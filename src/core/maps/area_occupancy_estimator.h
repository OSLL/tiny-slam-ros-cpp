#ifndef __AREA_OCCUPANCY_ESTIMATOR_H
#define __AREA_OCCUPANCY_ESTIMATOR_H

#include <cassert>
#include <vector>
#include <type_traits>

#include "cell_occupancy_estimator.h"

/*!
 * \brief A strategy that estimates a grid cell's occupancy based on how
 *        a laser beam passes through the cell.
 *
 * The estimation is based on a ratio between cell's chunks produced by
 * cutting the cell with a laser beam.
 */
class AreaOccupancyEstimator : public CellOccupancyEstimator {
private: // types

public: //methods

  /*!
   * Initializes the estimator with base probabilities.
   * \param[in] occ, empty - base probabilities of occupied/empty cells.
   */
  AreaOccupancyEstimator(double occ, double empty) :
    CellOccupancyEstimator(occ, empty) {}

  /*!
   * Finds the probability of a cell to be occupied based on chunk's areas.
   */
  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    Occupancy result(0.0, 0.0);

    bool beam_is_a_point = equal(beam.x_st, beam.x_end)
                        && equal(beam.y_st, beam.y_end);
    bool beam_intersects_cell = beam.intersects(cell_bnds);
    bool beam_escapes_occ_cell = !cell_bnds.is_inside(beam.x_end, beam.y_end)
                                 && is_occ;
    bool beam_escapes_from_border = (-beam).encounters(cell_bnds);

    if (beam_is_a_point || !beam_intersects_cell ||
        beam_escapes_occ_cell || beam_escapes_from_border) {
      result.invalidate();
      return result;
    }

    if (beam.encounters(cell_bnds)){
      return is_occ ? Occupancy(1.0, 1.0) : Occupancy(base_empty_prob(), 0.5);
    }

    Beam remote_beam = beam.move_start_out(cell_bnds);

    Intersections intrs = find_intersections(remote_beam, cell_bnds, is_occ);
    double chunk_area = compute_chunk_area(remote_beam, cell_bnds, is_occ,
                                                                        intrs);
    double area_ratio = chunk_area/cell_bnds.area();
    result = estimate_occupancy(chunk_area, cell_bnds.area(), is_occ);

    area_ratio = (0.5 < area_ratio) ? (1 - area_ratio) : area_ratio;

    if (remote_beam.touches_rect(cell_bnds)) {
      result.estimation_quality = 0.01;
    }
    else if (remote_beam.reaches_border_passing_through(cell_bnds)) {
      result.prob_occ = base_empty_prob();
      result.estimation_quality = area_ratio;
    }

    if (equal(result.prob_occ, base_empty_prob()) && is_occ) {
      result.invalidate();
    }

    return result;
  }

private: // methods
  Intersections find_intersections(const Beam &beam,
                                   const Rectangle &bnds, bool is_occ) {

    // if the cell is occupied, rotate ray around beam by 90 degrees
    Ray ray = is_occ ?
      Ray(beam.x_end, beam.y_st - beam.y_end,
          beam.y_end, beam.x_end - beam.x_st) :
      Ray(beam.x_st, beam.x_end - beam.x_st,
          beam.y_st, beam.y_end - beam.y_st);

    return ray.find_intersections(bnds);
  }

  double compute_chunk_area(const Beam &beam, const Rectangle &bnds,
                            bool is_occ,
                            const std::vector<Intersection> inters) {
    if (inters.size() == 0) {
      // TODO: deal with line approx introduced by Bresenham
      return bnds.area() / 2;
    }

    assert(inters.size() == 2 || inters.size() == 4);
    if (inters.size() == 4) {
      // TODO: generalize spec. case. Looks like ray is a cells diagonal
      return (bnds.top - bnds.bot) * (bnds.right - bnds.left) / 2;
    }

    double corner_x = 0, corner_y = 0, area = 0;
    double chunk_is_triangle = inters[0].is_horiz() ^ inters[1].is_horiz();
    if (chunk_is_triangle) {
      // determine "base" corner (corner of cell that
      // is also a corner of the triangle
      for (auto &inter : inters) {
        switch (inter.location) {
        case IntersLocation::Bot: corner_y = bnds.bot; break;
        case IntersLocation::Top: corner_y = bnds.top; break;
        case IntersLocation::Left: corner_x = bnds.left; break;
        case IntersLocation::Right: corner_x = bnds.right; break;
        }
      }
      // calculate triange area
      area = 0.5;
      for (auto &inter : inters) {
        if (inter.is_horiz()) {
          area *= inter.x - corner_x;
        } else {
          area *= inter.y - corner_y;
        }
      }
    } else {
      // chunk is trapezoid
      // corner choise doesn't matter, so pick bottom-left one.
      corner_x = bnds.bot, corner_y = bnds.left;
      double base_sum = 0;
      for (auto &inter : inters) {
        if (inter.is_horiz()) {
          base_sum += inter.x - corner_x;
        } else {
          base_sum += inter.y - corner_y;
        }
      }
      // NOTE: cell is supposed to be a square
      area = 0.5 * (bnds.top - bnds.bot) * base_sum;
    }
    if (is_occ &&
        are_on_the_same_side(inters[0].x, inters[0].y, inters[1].x, inters[1].y,
                             beam.x_st, beam.y_st, corner_x, corner_y)) {
      area = bnds.area() - area;
    }
    return area;
  }

  bool are_on_the_same_side(double line_x1, double line_y1,
                            double line_x2, double line_y2,
                            double x1, double y1, double x2, double y2) {
    double dx = line_x2 - line_x1, dy = line_y2 - line_y1;
    return 0 < (dy*y1 - dx*x1 + dy*x1 - dx*y1) *
               (dy*y2 - dx*x2 + dy*x2 - dx*y2);
  }

  Occupancy estimate_occupancy(double chunk_area, double total_area,
                               bool is_occ) {
    double area_rate = chunk_area / total_area;
    if (is_occ) {
      // Far ToDo: think about experiment quality metric for an occupied case.
      return Occupancy{area_rate, 1.0};
    } else {
      if (0.5 < area_rate) {
        area_rate = 1 - area_rate;
      }
      return Occupancy{base_empty_prob(), area_rate};
    }
  }
};

#endif
