/**
 * \file
 * \brief Defines some simple geometry structs and classes.
 * There are struct Rectangle, struct DiscretePoint2D, class DiscreteLine2D.
 */

#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <vector>
#include <cmath>


#define EQ_DOUBLE(a, b)                         \
  (std::abs((a) - (b)) < 0.0001)

/**
 * \brief Defines an axis-aligned rectangle.
 */
struct Rectangle {

  ///Creates a rectangle with zero area in point (0,0).
  Rectangle() : Rectangle(0, 0, 0, 0) {}

  /**
   * Creates a rectangle where all contained points are bounded in limits
   * l < x < r, b < y < t.
   * \param b The bottom of a rectangle.
   * \param t The top of a rectangle.
   * \param l The left side of a rectangle.
   * \param r The right side of a rectangle.
   */
  Rectangle(double b, double t, double l, double r) :
    bot(b), top(t), left(l), right(r) {}

  /**
   * Tests whether a rectangle contains a point.
   * \param x,y Coordinates of a target point.
   * \return True if the rectangle contains a point and False otherwise.
   */
  bool does_contain(double x, double y) const {
    return ((bot < y) && (y < top)) && ((left < x) && (x < right));
  }

  /**
   * Returns an area of a rectangle.
   * \return Value of an area.
   */
  double area() const {
    return (top - bot)*(right - left);
  }

  double bot,   ///< The bottom of a rectangle.
         top,   ///< The top of a rectangle.
         left,  ///< The left side of a rectangle.
         right; ///< The right side of a rectangle.
};

/**
 * \brief Defines a point with integer coordinates on a plane.
 */
struct DiscretePoint2D {
public:

  /**
   * Initializes a point in cartesian coordinates.
   * \param x_coord,y_coord Coordinates of a point on a plane.
   */
  DiscretePoint2D(int x_coord, int y_coord):
    x{x_coord}, y{y_coord} {}
  // TODO: mv (!!), cpy ctors
  int x, y; ///< Coordinates of point

  /// Operator of points summation by element-wise addition.
  DiscretePoint2D operator+(const DiscretePoint2D &p) const {
    return DiscretePoint2D(x + p.x, y + p.y);
  }

  /// Returns additive inverse of the point.
  DiscretePoint2D operator-() const {
    return DiscretePoint2D(-x, -y);
  }

  /**
   * Calculates distance from this point to a given one.
   * \param pt The end point to calculate distance.
   * \return The distance between this point and the end point.
   */
  double dist_sq(const DiscretePoint2D &pt) const {
    return std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2);
  }
};

/**
 * \brief Defines a line segment on a plane.
 */
class DiscreteLine2D {
  using Point = DiscretePoint2D;
public: // methods

  /**
   * Initializes a segment with end points.
   * \param start Beginning of a segment.
   * \param end Ending of a segment.
   */
  DiscreteLine2D(const Point &start, const Point &end) {
    generatePointsWithBresenham(start.x, start.y, end.x, end.y);
  }
  /// Returns the line's component points
  const std::vector<Point>& points() const { return _points; }
private:

  /**
   * Creates a line segment on a grid with a Bresenham algorithm;
   * if the World consists of cells, it is required to transform coordinates
   * of segment in a view that is useful for this representation of the world;
   * the result can be obtained by points() method.
   * \param x1,y1 Coordinates of the beginning of a line segment.
   * \param x2,y2 Coordinates of the ending of a line segment.
   */
  void generatePointsWithBresenham(int x1, int y1, int x2, int y2) {
    // TODO: copypasted from
    //   http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
    //   review and simplification are required

    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) * 2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;

    _points.push_back(Point(x1, y1));

    if (delta_x >= delta_y) {
      // error may go below zero
      int error(delta_y - (delta_x >> 1));
      while (x1 != x2) {
        if ((0 <= error) && (error || (0 < ix))) {
          error -= delta_x;
          y1 += iy;
        }
        // else do nothing
        error += delta_y;
        x1 += ix;
        _points.push_back(Point(x1, y1));
      }
    }
    else {
      // error may go below zero
      int error(delta_x - (delta_y >> 1));

      while (y1 != y2) {
        if ((0 <= error) && (error || (0 < iy))) {
          error -= delta_y;
          x1 += ix;
        }
        // else do nothing
        error += delta_x;
        y1 += iy;
        _points.push_back(Point(x1, y1));
      }
    }
  }
private: // fields
  std::vector<Point> _points;
};

#endif
