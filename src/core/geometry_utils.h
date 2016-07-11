#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <vector>
#include <cmath>


#define EQ_DOUBLE(a, b)                         \
  (std::abs((a) - (b)) < 0.0001)

struct Rectangle {
  Rectangle() : Rectangle(0, 0, 0, 0) {}
  Rectangle(double b, double t, double l, double r) :
    bot(b), top(t), left(l), right(r) {}

  bool does_contain(double x, double y) const {
    return ((bot < y) && (y < top)) && ((left < x) && (x < right));
  }

  double area() const {
    return (top - bot)*(right - left);
  }

  double bot, top, left, right;
};

struct DiscretePoint2D {
public:
  DiscretePoint2D(int x_coord, int y_coord):
    x{x_coord}, y{y_coord} {}
  // TODO: mv (!!), cpy ctors
  int x, y;

  DiscretePoint2D operator+(const DiscretePoint2D &p) const {
    return DiscretePoint2D(x + p.x, y + p.y);
  }

  DiscretePoint2D operator-() const {
    return DiscretePoint2D(-x, -y);
  }

  double dist_sq(const DiscretePoint2D &pt) const {
    return std::pow(x - pt.x, 2) + std::pow(y - pt.y, 2);
  }
};

class DiscreteLine2D {
  using Point = DiscretePoint2D;
public: // methods
  DiscreteLine2D(const Point &start, const Point &end) {
    generatePointsWithBresenham(start.x, start.y, end.x, end.y);
  }
  const std::vector<Point>& points() const { return _points; }
private:
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
