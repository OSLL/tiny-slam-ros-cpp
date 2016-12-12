#ifndef _MAP_TEST_H
#define _MAP_TEST_H

#include "../../src/core/maps/grid_map.h"
#include "../../src/tiny_slam/tiny_grid_cells.h"

class MapParams {
public:
  MapParams (int height = 0, int width = 0, int map_center_x = 0, int map_center_y = 0) :
             _height(height), _width(width), 
             _map_center_x(map_center_x), _map_center_y(map_center_y) {} 

  friend bool operator==(const MapParams &left, const MapParams &right);

  int _height;
  int _width;
  int _map_center_x;
  int _map_center_y;
};

bool operator==(const MapParams &left, const MapParams &right) {
  return (left._height == right._height) && (left._map_center_x == right._map_center_x)
       &&(left._width  == right._width)  && (left._map_center_y == right._map_center_y);
}

bool test_CellValueNotChange(GridMap map) {
  bool is_center_x_shifted = (map.map_center_x() == 0),
       is_center_y_shifted = (map.map_center_y() == 0);

  int x0 = is_center_x_shifted ? 0 :
           (map.map_center_x() - 1  - map.width());
  int y0 = is_center_y_shifted ? 0 :
           (map.map_center_y() - 1 - map.height());
  int i = 0, j = 0;

  for (i = x0; i < map.width(); ++i) {
    for (j = y0; j < map.height(); ++j) {
      if (map.cell_value({i, j}) != 0.5) {
        return false;
      }
    }
  }

  return true;
}

#endif
