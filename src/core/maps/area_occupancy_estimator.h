/*!
 * \file
 * \bried Description of class file (AreaOccupancyEstimator is inherited from CellOccupancyEstimator)
 *
 * This file includes one class AreaOccupancyEstimator which presents a special way to calculate occupancy of a cell
 * more informative than (is occupied/ is not occupied)
 */
#ifndef __AREA_OCCUPANCY_ESTIMATOR_H
#define __AREA_OCCUPANCY_ESTIMATOR_H

#include <cassert>
#include <vector>
#include <type_traits>

#include "cell_occupancy_estimator.h"
/*!
 * \brief Derived class from CellOccupancyEstimator to calculate the value of occupancy of one cell
 *
 * This class contains such structures as: Intersection (to store data of the intersection with cell bounds place)
 * and Ray (built from data \f$(x_0,y_0)\f$ and \f$(\Delta x, \Delta y)\f$),
 * and also enum class IntersLocation (to store general data about intersection place (on the left side, right side etc.)).
 * This class appears to make a rule to calculate the value of cell occupancy.
 * It takes the part of cell with is cut from the laser beam (ray cuts the cell on two parts) and calculates a ratio between the smallest and the biggest parts.
 */
class AreaOccupancyEstimator : public CellOccupancyEstimator {
private: // types
  /*!
   * \brief enumeration to generally locate a cross point (between laser beam and cell bounds)
   */
  enum class IntersLocation : char {
    Bot = 0,  ///< shows that the intersection locates on the bottom of rectangle
    Left = 1, ///< shows that the intersection locates on the left side of rectangle
	Top = 2,  ///< shows that the intersection locates on the top of rectangle
	Right = 3 ///< shows that the intersection locates on the left side of rectangle
  };

  /*!
   * \brief Structure collects the location of intersection with information about the \f$(x,y)\f$ position
   * and generally location (on the top, on the left side etc)
   */
  struct Intersection {
	/*!
	 * Parameterized constructor sets all data members
	 * \param[in] loc - where did intersection happen (bot, left, right, top)
	 * \param[in] inters_x, inters_y - the \f$x\f$ and \f$y\f$ coordinate of intersection
	 */
    Intersection(IntersLocation loc, double inters_x, double inters_y) :
      location(loc), x(inters_x), y(inters_y) {}

    IntersLocation location; ///< where did intersection happen (bot, left, right, top)
    /*!
     * Function shows that intersection locates on horizontal line
     * \return boolean value that \code location == IntersLocation::Bot || location == IntersLocation::Top \encode
     */
    bool is_horiz() const {
      return location == IntersLocation::Bot || location == IntersLocation::Top;
    }
    double x, y; ///< the \f$x\f$ and \f$y\f$ coordinate of intersection
  };

  using Intersections = std::vector<Intersection>;

  /*!
   * \brief Structure which define a ray in the space with beginning point and sift values
   * \f$\Delta x\f$ and \f$\Delta y\f$
   */
  struct Ray { // in parametric form
	/*!
	 * Parameterized constructor sets all data members
	 * \param[in] x_s, y_s - \f$(x, y)\f$ coordinate of the ray beginning
	 * \param[in] x_d, y_d - \f$(\Delta x, \Delta y)\f$ sift values to make a ray direction
	 */
    Ray(double x_s, double x_d, double y_s, double y_d) :
      x_st(x_s), x_delta(x_d), y_st(y_s), y_delta(y_d) {}

    double x_st, x_delta;  ///< the \f$x\f$ coordinate and shift
    double y_st, y_delta;  ///< the \f$y\f$ coordinate and shift

    /*!
     * Function finds if there is an intersection with this ray and horizontal segment given as input variable
     * \param[in] st_x, end_x - the \f$\x\f$ beginning and ending coordinates of input segment
     * \param[in] y - the \f$y\f$ coordinate of input segment
     * \param[in] loc - the place where intersection happens
     * \param[in,out] consumer - vector of all points-intersections
     */
    void intersect_horiz_segm(double st_x, double end_x, double y,
                              IntersLocation loc, Intersections &consumer) {
      if (EQ_DOUBLE(y_delta, 0))
        return;

      double inters_alpha = (y - y_st) / y_delta;
      double inters_x = x_st + inters_alpha * x_delta;
      if (inters_x < st_x || end_x < inters_x) // out of segment bounds
        return;

      consumer.push_back(Intersection(loc, inters_x, y));
    }

    /*!
     * Function finds if there is an intersection with this ray and vertical segment given as input variable
     * \param[in] st_y, end_y - the \f$\y\f$ beginning and ending coordinates of input segment
     * \param[in] x - the \f$x\f$ coordinate of input segment
     * \param[in] loc - the place where intersection happens
     * \param[in,out] consumer - vector of all points-intersections
     */
    void intersect_vert_segm(double st_y, double end_y, double x,
                             IntersLocation loc, Intersections &consumer) {
      if (EQ_DOUBLE(x_delta, 0))
        return;

      double inters_alpha = (x - x_st) / x_delta;
      double inters_y = y_st + inters_alpha * y_delta;
      if (inters_y < st_y || end_y < inters_y) // out of segment bounds
        return;
      consumer.push_back(Intersection(loc, x, inters_y));
    }
  };

public: //methods

  /*!
  	 * Parameterized constructor sets all data members
  	 * \param[in] occ, empty - values for base class CellOccupancyEstimator
  	 */
  AreaOccupancyEstimator(double occ, double empty) :
    CellOccupancyEstimator(occ, empty) {}

  virtual Occupancy estimate_occupancy(const Beam &beam,
                                       const Rectangle &cell_bnds,
                                       bool is_occ) override {
    Intersections intrs = find_intersections(beam, cell_bnds, is_occ);
    double chunk_area = compute_chunk_area(beam, cell_bnds, is_occ, intrs);
    return estimate_occupancy(chunk_area, cell_bnds.area(), is_occ);
  }

private: // methods
  /*!
   * Function calculates intersections of the input beam with all bounds of input rectangle
   *
   * The beam crosses cell bounds in two points if the cell is not occupied and in one point if not
   * (and "stops" on the obstacle). So in the second way it could be said that the obstacle is perpendicular to the laser beam.
   * This method returns the intersection points laser beam with cell bounds if this cell there is no obstacle there,
   * and the point where this obstacle could cross the cell bounds if there is the obstacle.
   *
   * \param[in] beam - input laser beam
   * \param[in] bnds - bound of cell area
   * \param[in] is_occ - flag is this cell occupied or not
   * \return the vector of intersections if they are exist
   */
  Intersections find_intersections(const Beam &beam,
                                   const Rectangle &bnds, bool is_occ) {
    Intersections intersections;
    // if the cell is occupied, rotate ray around beam by 90 degrees
    Ray ray = is_occ ?
      Ray(beam.x_end, beam.y_st - beam.y_end,
          beam.y_end, beam.x_end - beam.x_st) :
      Ray(beam.x_st, beam.x_end - beam.x_st,
          beam.y_st, beam.y_end - beam.y_st);

    ray.intersect_horiz_segm(bnds.left, bnds.right, bnds.top,
                             IntersLocation::Top, intersections);
    ray.intersect_horiz_segm(bnds.left, bnds.right, bnds.bot,
                             IntersLocation::Bot, intersections);
    ray.intersect_vert_segm(bnds.bot, bnds.top, bnds.left,
                            IntersLocation::Left, intersections);
    ray.intersect_vert_segm(bnds.bot, bnds.top, bnds.right,
                            IntersLocation::Right, intersections);
    return intersections;
  }

  /*!
   * Function computes the area of a occupied chunk.
   * The laser beam (or bound of possible obstacle) could cut the cell on triangle and trapezoid or on two trapezoids
   * so the returned value "area" is the area of one cut parts of cell
   * \param[in] beam - the laser ray
   * \param[in] bnds - the bounds of a cell
   * \param[in] is_occ - flag that this cell is occupied
   * \param[in] inters - the array of points - intersections wall with cell bounds
   * \return the area of cell chunk which is cut by laser beam (or obstacle bound)
   */
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
    /* inters[0].is_horis | inters[1].is_horiz | XOR (^)
     * -------------------+--------------------+---------
     *        false       |        false       |  false
     * -------------------+--------------------+---------
     *        false       |        true        |  true   - one intersection happens on horizontal line but another
     * -------------------+--------------------+---------      (so chunk is a triangle)
     *        true        |        false       |  true   - one intersection happens on horizontal line but another
     * -------------------+--------------------+---------      (so chunk is a triangle)
     *        true        |        true        |  false
     */
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

  /*!
   * Function answers on a question are pints \f$(x_1,y_1)\f$ and \f$(x_2,y_2)\f$ locate on the same side of line based on points \f$(x_{line 1},y_{line 1})\f$ and \f$(x_{line 2},y_{line 2})\f$
   * \param[in] line_x1, line_y1, line_x2, line_y2 - coordinates \f$(x_{line 1},y_{line 1})\f$ and \f$(x_{line 2},y_{line 2})\f$ of line
   * \param[in] x1,y1,x2,y2 - coordinates \f$(x_1,y_1)\f$ and \f$(x_2,y_2)\f$ of interesting two points
   * \return do point locate on the same side from line or don't
   */
  bool are_on_the_same_side(double line_x1, double line_y1,
                            double line_x2, double line_y2,
                            double x1, double y1, double x2, double y2) {
    double dx = line_x2 - line_x1, dy = line_y2 - line_y1;
    return 0 < (dy*y1 - dx*x1 + dy*x1 - dx*y1) *
               (dy*y2 - dx*x2 + dy*x2 - dx*y2);
  }

  /*!
   * Function gives the value of possibility of current cell be occupied
   * \param[in] chunk_area - the area of chunk cell cut by laser beam
   * \param[in] total_area - the area of cell
   * \param[in] is_occ - flag illustrated that this cell is occupied (there is an obstacle in this cell)
   * \return the probability of current cell to be occupied
   */
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
