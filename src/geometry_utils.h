/*
Copyright (c) 2016 JetBrains Research, Mobile Robot Algorithms Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy of this 
software and associated documentation files (the "Software"), to deal in the Software 
without restriction, including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <vector>
#include <cmath>


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
